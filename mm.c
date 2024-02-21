// v8 final 240220
// score = 58 + 40


// Results for mm malloc:
// trace  valid  util     ops      secs  Kops
//  0       yes   99%    5694  0.000421 13525
//  1       yes   98%    5848  0.000362 16164
//  2       yes   99%    6648  0.000812  8188
//  3       yes   99%    5380  0.000813  6617
//  4       yes   99%   14400  0.000219 65753
//  5       yes   95%    4800  0.010560   455
//  6       yes   94%    4800  0.010340   464
//  7       yes   97%   12000  0.000666 18021
//  8       yes   94%   24000  0.004102  5851
//  9       yes  100%   14401  0.000165 87226
// 10       yes   84%   14401  0.000191 75556
// Total          96%  112372  0.028650  3922

// -----------[BENCH]-----------
//  mm_malloc    |  1.0073900s
//  mm_free      |  0.4873080s
//  mm_realloc   |  0.0861820s
// ----------SUBROUTINE---------
//  coalesce_all |  0.1017780s
//  extend_heap  |  0.0648740s
// -----------------------------

////////////////////////////////////// NOTE /////////////////////////////////////////////////

// *[GENERAL]*
// footer 없음
// malloc/realloc중 공간이 모자랄 때 한 번에 coalesce
// 마지막으로 coalesce 한 후 한 번도 free 호출이 없었다면 굳이 coalesce 하지 않음
// realloc시 마지막 블록인 경우 heap을 연장하여 공간 확보


// *[FREE LIST]*
// explicit free list
// (4)[헤더] (4)[prev] (4)[next]
// best fit
// doubly linked list (순환형)
// FIFO

// segregated list
// size class: 16~64, 65~256, 257~ (코드에서 bucket으로 칭함)
// 힙의 prologue 블록에 각 size class의 루트에 해당하는 노드가 저장됨
// (4)[헤더]|(4)[prev] (4)[next]|(4)[prev] (4)[next]|(4)[prev] (4)[next]|(4)[prev] (4)[next]

// malloc 호출 시 해당 size class의 free 블록에 할당됨
// 특정 size class에 속한 블록은 다른 size class에게 할당될 수 없음
// 헤더의 extra bit로 size class를 표시 (01, 10, 11)

// size class별로 다른 chunk size 사용 (~64: 512, ~256: 2312, 257~: fit)
// 공간이 부족하면 해당하는 chunk size 크기의 free 블록을 heap 연장으로 생성
// 256보다 큰 malloc 호출은 chunk size 없이 크기에 맞게 heap 연장


// *[MICRO CELL]*
// 1~16byte의 데이터는 특수한 micro cell 자료구조에 저장
// 개별 헤더 없이 64개의 16byte 데이터를 한 번에 저장하는 블록

// (4)[헤더] (4)[prev] (4)[next] (8)[cell list] (16)[payload] (16)[payload] (16)[payload]...
// cell list는 64개의 비트(long long unsigned)로 각 cell이 할당되었는지 여부를 표시
// prev, next는 할당 여부에 관계 없이 항상 존재
// 헤더의 size class는 00으로 표시
// 다른 size class와 마찬가지로 prologue 블록에 micro cell를 저장하는 list의 루트가 있음

// free 호출 시, 주소의 헤더를 확인할 수 없으므로 모든 micro cell 블록을 확인해야 함

// 모든 cell이 free된 micro cell 블록은 크기에 맞는 size class의 free 블록으로 변환됨

/////////////////////////////////////////////////////////////////////////////////////////////



#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>

#include "mm.h"
#include "memlib.h"

team_t team = {
    "KMS",
    "Minsoo Kang",
    "mskang@mail",
    "",
    ""
};


// 정렬 byte 수 (4byte에서 테스트되지 않음)
#define ALIGNMENT 8
// 8의 배수로 반올림 (ALIGNMENT가 8일때만 작동하는 코드)
#define ALIGN(size) (((size) + (ALIGNMENT-1)) & ~0x7)

// 헤더 매크로
#define HDR_SIZE 4 // 헤더 크기
#define MIN_BLK_SIZE ALIGN( HDR_SIZE + 2 * sizeof(void*) ) // 최소 블록 크기
#define GET_HDR(p) ( (int*) ( (void*) (p) - HDR_SIZE) ) // 블록 헤더 주소 계산

// Size Class
#define BCKT_NUM 4 // size class의 수: 헤더의 extra bit을 사용하므로 4가 최대
static size_t bucket_thr[BCKT_NUM] = {16, 64, 256, (size_t) -1}; // size class 구분
static size_t chunk_size[BCKT_NUM] = {-1, 512, 2312, -1}; // size class의 chunk size

// Micro Cell
#define MICRO_SIZE 16 // 16byte 이하의 데이터는 모두 micro cell 블록에 저장됨
#define MCELL_CNT 64 // micro cell block에 들어갈 cell개수
#define FULL_CELL_LIST (unsigned long long) -1 // 가득 찬 cell list (MCELL_CNT = 64 인 경우 사용)
// #define FULL_CELL_LIST ( ( ((unsigned long long) 1) << MCELL_CNT ) -1 ) // (MCELL_CNT < 64 인 경우 사용)

// micro cell의 크기: 헤더 + next/prev + cell list(8byte) + 16 * cell개수
#define MCELL_SIZE ALIGN( HDR_SIZE + ALIGN(2 * sizeof(void*)) + sizeof(unsigned long long) + 16 * MCELL_CNT )
// micro cell의 cell list위치
#define GET_MCELL_LIST(p) ( (unsigned long long*) ( (int) (p) + ALIGN(2 * sizeof(void*)) ) )
// p위치의 micro cell 블록에서 idx번째 cell의 주소
#define GET_CELL_P(p, idx) ( (void*) ( (int) (p) + ALIGN(2 * sizeof(void*)) + sizeof(unsigned long long) + 16 * idx ) )


static void *h_root_p; // prologue 블록의 주소
static void *h_start_p; // prologue 블록 직후 블록의 주소
static void *h_end_p; // epilogue 블록의 주소

static char just_coalesced = 1; // 마지막 coalesce_all 호출 이후로 free 호출이 한 번이라도 됐으면 0


//-----------------------------HDR FUNC--------------------------------------
// p위치의 직전에 size, bucket 번호, allocate여부를 표시하는 4바이트 헤더를 입력
// bucket_idx는 0~3 사이의 정수여야 함
// is_allocated는 0 또는 1이어야 함
void tag_header(int *p, size_t size, int bucket_idx, char is_allocated) {
    *GET_HDR(p) = (int) size | (bucket_idx << 1) | is_allocated;
}

// p위치의 헤더의 크기 정보를 반환
size_t read_header_size(void *p) {
    return (size_t) *GET_HDR(p) & (~7);
}

// p위치의 헤더의 bucket 번호를 반환 (2, 3번 비트를 읽음)
int read_header_bi(void *p) {
    return ( *GET_HDR(p) & 6 ) >> 1;
}

// p위치의 헤더의 allocate여부 정보를 반환
char read_header_allo(void *p) {
    return *GET_HDR(p) & 1;
}


//-----------------------------BCKT FUNC--------------------------------------
// segregated list의 idx번째 bucket의 root 위치를 반환
void *get_bucket_root(int idx) {
    return h_root_p + idx * 2 * sizeof(void*);
}

// size 크기의 블록이 몇 번째 bucket에 속하는지 반환
int get_bucket_idx(size_t size) {
    // bucket 0은 16byte micro cell에서만 사용되므로 bucket 1부터 고려
    for (int idx = 1; idx < BCKT_NUM; idx++) {
        if (size <= bucket_thr[idx]) {
            return idx;
        }
    }
    return -1;
}


//-----------------------------LIST FUNC--------------------------------------
// p위치 free된 블록의 직전 free블록 주소 쓰기
void set_prev(void *p, void *prev) {
    *( (void**) p) = prev;
}

// p위치 free된 블록의 직후 free블록 주소 쓰기
void set_next(void *p, void *next) {
    *( (void**) ( p + sizeof(void*) ) ) = next;
}

// p위치 free된 블록의 직전 free블록 주소 읽기
void *read_prev(void *p) {
    return *( (void**) p);
}

// p위치 free된 블록의 직후 free블록 주소 읽기
void *read_next(void *p) {
    return *( (void**) (p + sizeof(void*)) );
}

// p위치의 free된 블록을 free list에 삽입 (root와 그 이전 node 사이에)
void insert_node(void *p) {
    void *bucket_root = get_bucket_root(read_header_bi(p));

    // FIFO
    set_next(read_prev(bucket_root), p); // prev => p
    set_prev(p, read_prev(bucket_root)); // p => prev

    set_prev(bucket_root, p); // root => p
    set_next(p, bucket_root); // p => root
}

// p위치의 free된 블록을 free list에서 삭제
void remove_node(void *p) {
    set_prev(read_next(p), read_prev(p)); // next => prev
    set_next(read_prev(p), read_next(p)); // prev => next
}


//-----------------------------UTIL FUNC--------------------------------------
// p위치의 블록을 new_size 크기의 블록과 나머지로 분할 (가능한 경우)
void insert_block(void *p, size_t new_size) {
    size_t old_size = read_header_size(p);
    int bucket_idx = read_header_bi(p);
    
    if (old_size < new_size) {
        // 기존 블록이 더 작으면 에러
        printf("insert_block(): original block is smaller than new block\n");
    } else if (old_size >= new_size + MIN_BLK_SIZE) {
        // 기존 블록 크기가 최소 블록 크기보다 더 크게 차이나는 경우에만 쪼개기
        tag_header(p, new_size, bucket_idx, read_header_allo(p)); // new_size 크기의 블록
        tag_header(p + new_size, old_size - new_size, bucket_idx, 0); // 쪼개진 블록

        insert_node(p + new_size); // 쪼개진 블록을 free list에 삽입
    }
}

// new_size(헤더 포함 및 정렬된 크기)를 받아 best fit 블록을 반환
void *find_fit(size_t new_size) {
    void *root_p = get_bucket_root(get_bucket_idx(new_size));
    void *search_p = read_next(root_p);

    size_t min_diff = (size_t) -1; // 최대 정수
    void *min_p = NULL;
    size_t cur_diff = 0;

    while (search_p != root_p) {
        // best fit 탐색
        // new_size에 해당하는 bucket에서만 탐색
        if (!read_header_allo(search_p) && read_header_size(search_p) >= new_size) {
            cur_diff = read_header_size(search_p) - new_size;
            if (cur_diff == 0) {
                // 완벽한 블록 발견; 즉시 반환
                return search_p;
            } else {
                if (cur_diff < min_diff) {
                    min_diff = cur_diff;
                    min_p = search_p;
                }
            }
        }
        search_p = read_next(search_p); // free list의 다음 node로 이동
    }

    // 탐색한 best fit 블록이 min_p에 저장됨
    if (min_p != NULL) {
        insert_block(min_p, new_size); // 쪼개기 시도
    }

    return min_p;
}

// p위치의 블록을 이후의 연속된 free블록들과 연결 (같은 size class일 때만)
// p위치의 블록이 할당된 블록이어도 호출 가능 (realloc에서 사용)
void coalesce_one(void *p) {
    if (read_header_bi(p) == 0) {
        // micro cell 블록은 coalesce 하지 않음
        printf("coalesce_one(%p): received micro cell block!\n", p);
        return;
    }

    size_t tot_free_size = read_header_size(p); // p위치 블록 크기
    void *search_p = p + read_header_size(p); // 직후 블록
    int bucket_idx = read_header_bi(p);

    while (!read_header_allo(search_p) && (read_header_bi(search_p) == bucket_idx)) {
        // 연속된 블록 발견
        tot_free_size += read_header_size(search_p); // 연결된 후의 크기에 합산
        remove_node(search_p); // free list에서 제거
        search_p += read_header_size(search_p);
    }
    // 병합된 size를 업데이트 (bucket_idx, is_allocated는 그대로)
    tag_header(p, tot_free_size, read_header_bi(p), read_header_allo(p));
}

// 힙의 모든 free block을 coalesce
void coalesce_all() {
    void *search_p = h_start_p;

    while (search_p < h_end_p) {
        if (!read_header_allo(search_p) && read_header_bi(search_p) != 0) {
            // free 블록 발견: 이어지는 free 블록을 병합 (같은 bucket끼리만)
            // micro cell 블록은 coalesce하지 않음
            coalesce_one(search_p);
        }
        search_p += read_header_size(search_p);
    }

    just_coalesced = 1;
}

// chunk size만큼 heap을 확장시키고 기존의 h_end_p를 반환
// 빈 블록을 size에 따른 bucket에 귀속시킴
// 요청한 free 블록은 free list에 삽입하지 않음 (필요하다면 호출한 함수에서 삽입)
void *extend_heap(size_t size) {
    if (size < (size_t) MIN_BLK_SIZE) {
        printf("extend_heap(): extend length is smaller than minimum block size (%d byte)!\n", MIN_BLK_SIZE);
        return NULL;
    }

    void *ret_p = h_end_p;
    int bucket_idx = get_bucket_idx(size);
    
    if (size <= bucket_thr[2]) {
        // size class가 1 또는 2인 경우: chunk size 크기의 free 블록 생성
        mem_sbrk(chunk_size[bucket_idx]);

        tag_header(h_end_p, size, bucket_idx, 0); // 요청한 size만큼의 free block
        tag_header(h_end_p + size, chunk_size[bucket_idx] - size, bucket_idx, 0); // chunk에 속하는 나머지 블록
        insert_node(h_end_p + size);

        h_end_p += chunk_size[bucket_idx];
    } else {
        // size class가 3인 경우: 요청한 크기만큼의 free 블록 생성
        mem_sbrk(size);
        tag_header(h_end_p, size, bucket_idx, 0); // 요청한 size만큼의 free block
        h_end_p += size;
    }
    tag_header(h_end_p, 0, 0, 1); // update epilouge block

    return ret_p;
}


//-----------------------------M.CL FUNC--------------------------------------
// p위치의 micro cell 블록에 속하는, cp위치의 cell이 몇 번째 cell인지 계산해서 반환
int get_cell_idx(void *p, void *cp) {
    // 0번째 cell과의 주소 차이 / 16
    return (int) ( (cp - GET_CELL_P(p, 0)) >> 4 );
}

// p위치의 데이터가 micro cell 블록에 속하는지 확인
// 속한다면 해당 블록의 주소를 반환
// free 호출 시 사용됨
void *find_micro_header(void *p) {
    void *micro_h_p = h_root_p;
    micro_h_p = read_next(micro_h_p);

    while (micro_h_p != h_root_p) { // 모든 micro cell을 탐색
        if ( (micro_h_p < p) && (p < micro_h_p + read_header_size(micro_h_p) - HDR_SIZE) ) {
            return micro_h_p;
        }
        micro_h_p = read_next(micro_h_p);
    }
    return NULL;
}

// p위치의 micro cell 블록에서 빈 cell하나를 할당
void *micro_assign_cell(void *p) {
    for (int idx = 0; idx < MCELL_CNT; idx++) {
        if (( *GET_MCELL_LIST(p) & (((unsigned long long) 1) << idx) ) == 0) {
            // idx번째의 cell이 비었음
            *GET_MCELL_LIST(p) = ( *GET_MCELL_LIST(p) | ( ((unsigned long long) 1) << idx) ); // list 업데이트
            if (*GET_MCELL_LIST(p) == FULL_CELL_LIST) {
                // 모든 cell이 가득 참: 헤더 업데이트
                tag_header(p, read_header_size(p), 0, 1);
            }

            return GET_CELL_P(p, idx);
        }
    }

    printf("micro_assign_cell(%p): no available cells!\n", p); // 오류
    return NULL;
}

// micro cell 블록 내부의 16byte cell을 할당 후 주소를 반환
void *micro_malloc() {
    void *micro_h_p = h_root_p;
    micro_h_p = read_next(micro_h_p);

    // 1. 먼저 빈 cell이 있는지 탐색
    while (micro_h_p != h_root_p) { // 존재하는 모든 micro cell 블록을 탐색
        if (!read_header_allo(micro_h_p)) {
            // 헤더의 allo가 0임 (빈 cell이 최소 하나 존재)
            // 해당 cell을 할당
            void *ret_p = micro_assign_cell(micro_h_p);
            return ret_p;
        }
        micro_h_p = read_next(micro_h_p);
    }

    // 2. 빈 cell이 없으면 새로운 micro cell block 생성
    // (mm_malloc을 변형한 코드)
    micro_h_p = find_fit(MCELL_SIZE); // micro cell 블록을 삽입할 수 있는 공간을 탐색

    if (micro_h_p == NULL && !just_coalesced) {
        // 빈 블록 탐색 실패: coalesce 후 한 번 더 시도
        // 이미 coalesce된 상태라면 추가로 시도하지 않음
         coalesce_all();
        micro_h_p = find_fit(MCELL_SIZE);
    }

    if (micro_h_p != NULL) {
        // 탐색 성공
        remove_node(micro_h_p); // free list에서 삭제
        tag_header(micro_h_p, read_header_size(micro_h_p), 0, 0); // 헤더 업데이트
    } else {
        // 탐색 2차 실패: sbrk 호출 및 micro cell 블록 생성
        micro_h_p = h_end_p;
        mem_sbrk(MCELL_SIZE);
        tag_header(micro_h_p, MCELL_SIZE, 0, 0); // 헤더 업데이트
        h_end_p += MCELL_SIZE;
    }

    insert_node(micro_h_p); // free list에 삽입

    *GET_MCELL_LIST(micro_h_p) = (unsigned long long) 1; // cell list에서 0번 cell을 배정
    // 0번 cell의 주소 반환
    return GET_CELL_P(micro_h_p, 0);
}

// p위치의 micro cell 블록에 있는, cell_p 위치의 cell을 free
void micro_free(void *p, void *cell_p) {
    // list에서 idx를 삭제
    *GET_MCELL_LIST(p) = *GET_MCELL_LIST(p) & ~( ((unsigned long long) 1) << get_cell_idx(p, cell_p));
    tag_header(p, read_header_size(p), 0, 0); // is_allocated를 0으로 업데이트

    if (*GET_MCELL_LIST(p) == 0) {
        // 만약 모든 cell이 비었다면 micro cell 블록을 free
        remove_node(p); // free list에서 삭제

        // micro cell 블록의 크기에 해당하는 bucket에 free 블록으로 삽입
        tag_header(p, read_header_size(p), get_bucket_idx(read_header_size(p)), 0);
        insert_node(p);
    }
}


//-----------------------------DEBUG FUNC--------------------------------------
// DEBUG: 모든 블록 출력
void print_heap() {
    void *p = h_start_p;

    int idx = 0;

    printf("\nprinting all heap...\n");
    printf("  ID  |  ADDRESS   | SIZE |BI |AL | PREV ADRS  | NEXT ADRS  \n");
    printf("------|------------|------|---|---|------------|------------\n");
    printf(" PRLG | %p | %4d | %d | %d |          - |          - \n",h_root_p, read_header_size(h_root_p), read_header_bi(h_root_p), read_header_allo(h_root_p));
    
    void *root_p;
    for (int i = 0; i < BCKT_NUM; i++) {
        root_p = get_bucket_root(i);
        printf("  B%d  | %p |      |   |   | %p | %p \n", i, root_p, read_prev(root_p), read_next(root_p));
    }
    while (p < h_end_p) {
        if (read_header_bi(p) == 0) {
            printf(" %3d. | %p | %4d | %d | %d | %p | %p \n", idx, p, read_header_size(p), read_header_bi(p), read_header_allo(p), read_prev(p), read_next(p));
            printf("M.CELL| CELL LIST  |    - | - | - |        %16llx\n", *GET_MCELL_LIST(p));
        } else {
            if (read_header_allo(p)) {
                printf(" %3d. | %p | %4d | %d | %d |          - |          - \n", idx, p, read_header_size(p), read_header_bi(p), read_header_allo(p));
            } else {
                printf(" %3d. | %p | %4d | %d | %d | %p | %p \n", idx, p, read_header_size(p), read_header_bi(p), read_header_allo(p), read_prev(p), read_next(p));
            }
        }
        idx += 1;
        p += read_header_size(p);
    }
    printf(" EPLG | %p | %4d | %d | %d |          - |          - \n\n", h_end_p, read_header_size(h_end_p), read_header_bi(h_end_p), read_header_allo(h_end_p));
}

// DEBUG: 모든 블록 free
void free_all() {
    void *p = h_start_p;
    while (p < h_end_p) {
        if (read_header_allo(p)) {
            mm_free(p);
        }
        p += read_header_size(p);
    }
}

// DEBUG: idx번째 블록의 주소 반환
void *get_block(int idx) {
    void *p = h_start_p;
    for (int i = 0; i < idx; i++) {
        p += read_header_size(p);
        if (p >= h_end_p) {
            // idx번째 블록이 존재하지않음
            return NULL;
        }
    }
    return p;
}


//-----------------------------MAIN FUNC--------------------------------------
// 힙 초기화: prologue 블록 및 epilogue 블록 삽입
int mm_init(void)
{
    // prologue block 삽입 (헤더, bucket의 루트 노드 포함)
    h_root_p = (void*) ALIGN( (int) mem_heap_lo() + HDR_SIZE );
    tag_header(h_root_p, (size_t) ALIGN(HDR_SIZE + 2 * BCKT_NUM * sizeof(void*)), 0, 1);
    // bucket 개수만큼 root를 생성
    void *bucket_root_p;
    for (int i = 0; i < BCKT_NUM; i++) {
        // root의 prev와 next에 자기 자신을 연결
        bucket_root_p = get_bucket_root(i);
        set_prev(bucket_root_p, bucket_root_p);
        set_next(bucket_root_p, bucket_root_p);
    }
    
    // epilogue block 삽입 (크기가 0인 헤더)
    h_start_p = h_root_p + read_header_size(h_root_p);
    h_end_p = h_start_p;
    mem_sbrk(h_start_p - mem_heap_lo());
    tag_header(h_start_p, 0, 0, 1);

    return 0;
}

// 할당: size class에 해당되는 블록을 검색 또는 생성
void *mm_malloc(size_t size)
{
    if (size <= MICRO_SIZE) {
        // 16byte 이하의 요청이 들어오면 micro cell에 넣기위해 micro_malloc을 호출
        return micro_malloc();
    }
    
    size_t new_size = ALIGN(size + (size_t) HDR_SIZE); // 실제로 힙에 삽입될 블록의 크기

    if (new_size < (size_t) MIN_BLK_SIZE) {
        // 헤더와 주소를 넣기 위한 최소 크기를 보장
        new_size = (size_t) MIN_BLK_SIZE;
    }

    // 1. 사용 가능한 free 블록 검색
    void *ret_p = find_fit(new_size); // bucket에 해당되고 new_size에 맞는 블록을 탐색
    
    if (ret_p != NULL) {
        // 탐색 성공
        remove_node(ret_p); // free list에서 삭제
        tag_header(ret_p, read_header_size(ret_p), read_header_bi(ret_p), 1); // 할당 및 헤더 업데이트

        return ret_p;
    }
    
    // 2. 탐색 실패: coalesce 후 한 번 더 시도
    if (!just_coalesced) { // 이미 coalesce된 상태라면 추가로 시도하지 않음
        coalesce_all();

        ret_p = find_fit(new_size);
        if (ret_p != NULL) {
            // 탐색 성공
            remove_node(ret_p); // free list에서 삭제
            tag_header(ret_p, read_header_size(ret_p), read_header_bi(ret_p), 1); // 할당 및 헤더 업데이트

            return ret_p;
        }
    }

    // 3. 2차 실패: extend_heap 호출 후 할당
    ret_p = extend_heap(new_size);
    tag_header(ret_p, new_size, read_header_bi(ret_p), 1);
    
    return ret_p;
}

// 반환: 헤더 및 free list 업데이트
void mm_free(void *ptr)
{
    // 먼저 ptr이 micro cell 블록에 해당되는지 확인
    void *micro_h_p = find_micro_header(ptr);

    if (micro_h_p != NULL) {
        // ptr은 micro cell 블록에 속하는 주소이므로 micro_free를 호출
        micro_free(micro_h_p, ptr);
        return;
    }

    tag_header(ptr, read_header_size(ptr), read_header_bi(ptr), 0); // header를 free로
    insert_node(ptr); // free list에 삽입

    just_coalesced = 0; // coalesce가 필요하다고 표시
}

// 재할당: 블록 확장, 블록 생성, 다른 블록 검색을 통해 더 큰 용량 제공
void *mm_realloc(void *ptr, size_t size)
{
    // 먼저 ptr이 micro cell 블록에 해당되는지 확인
    void *micro_h_p = find_micro_header(ptr);

    if (micro_h_p != NULL) {
        // ptr은 micro cell 블록에 속하는 주소
        if (size > (size_t) MICRO_SIZE) {
            // 16byte를 초과해 재할당을 요청
            // micro cell에서 제거 후 malloc으로 새로 할당
            micro_free(micro_h_p, ptr);
            void *ret_p = mm_malloc(size);

            memcpy(ret_p, ptr, MICRO_SIZE); // 데이터 복사
            return ret_p;
        } else {
            // 16byte를 초과하지 않으므로 작업 필요 없음
            return ptr;
        }
    }
    
    size_t new_size = ALIGN(size + (size_t) HDR_SIZE);
    size_t old_size = read_header_size(ptr);

    if (old_size >= new_size) {
        // padding만으로 충분한 공간이 확보됨
        insert_block(ptr, new_size); // 쪼개기 시도
        return ptr;
    }

    // 이후 블록이 1. free 블록이거나 2. epilogue 블록인 경우로 분기
    if (!read_header_allo(ptr + old_size)) {
        // 1. 직후 블록이 free되어있음
        if (!just_coalesced) { // 필요 시 해당 블록 및 연속된 블록을 coalesce
            coalesce_one(ptr);
        }
        if (read_header_size(ptr) >= new_size) {
            // 충분한 공간을 확인함
            insert_block(ptr, new_size); // 필요 시 분할
            return ptr;
        }
    } else if (read_header_size(ptr + old_size) == 0) {
        // 2. epilogue 블록인 경우 sbrk를 호출해 공간 확보
        mem_sbrk((int) (new_size - old_size));
        tag_header(ptr, new_size, read_header_bi(ptr), 1);

        h_end_p += new_size - old_size;
        tag_header(h_end_p, 0, 0, 1); // update epilouge block

        return ptr;
    }

    // 모두 실패: 공간이 모자라므로 다른 블록을 탐색해야 함
    void *new_ptr = mm_malloc(size);
    memcpy(new_ptr, ptr, read_header_size(ptr) - (size_t) HDR_SIZE); // 데이터 복사
    mm_free(ptr); // 기존 블록은 free

    return new_ptr;
}
