#include "ubf.h"

#include "string.h"
#include "stdlib.h"

struct _ubf_t
{
    void* pbuff;//缓存区的首地址
    uint32_t num;//num个size字节的数据单元
    uint32_t size;
    uint32_t write;//待写入单元索引，(write-1)指向最新数据
    uint8_t isfull;//缓存器满标志
};

ubf_t ubf_create(uint32_t num, uint32_t size)
{
    ubf_t pubf = (ubf_t)malloc(sizeof(struct _ubf_t));
    memset(pubf, 0, sizeof(struct _ubf_t));
    if (pubf)
    {
        pubf->num = num;
        pubf->size = size;
        pubf->pbuff = calloc(num, size);//缓存器内存实体，同时清零
        return pubf;
    }
    return NULL;
}

void ubf_delete(ubf_t ubf)
{
    free(ubf);
}

uint8_t ubf_push(ubf_t ubf, const void* pdata)
{
    if (ubf == NULL)
        return 1;
    memcpy((uint8_t*)ubf->pbuff+ubf->write*ubf->size, pdata, ubf->size);
    if (++ubf->write >= ubf->num)
    {
        ubf->write = 0;
        ubf->isfull = 1;
    }
    return 0;
}

void* ubf_pop(ubf_t ubf, uint32_t k)
{
    void* res;
    if (ubf == NULL)//缓存器不存在
        res = NULL;
    else if (k < ubf->write)//向前读取
        res = (uint8_t*)ubf->pbuff + (ubf->write-1-k)*ubf->size;
    else if (k < ubf->num)//折回尾部读取
        res = (uint8_t*)ubf->pbuff + ((ubf->num - 1) - (k - ubf->write)) * ubf->size;
    else//超出缓存容量
        res = NULL;
    return res;
}

uint32_t ubf_get_stock(ubf_t ubf)
{
    uint32_t res;
    if (ubf == NULL)//缓存器不存在
        res = 0;
    if (ubf->isfull)//缓存器已满
        res = ubf->num;
    else
        res = ubf->write;
    return res;
}

void ubf_clear(ubf_t ubf)
{
    if (ubf == NULL)//缓存器不存在
        return;
    memset(ubf->pbuff, 0, ubf->num * ubf->size);
    ubf->write = 0;
}

/* 首元素为最新的数据 */
uint32_t ubf_pop_into_array_new2old(ubf_t ubf, void* pdata, uint32_t k, uint32_t num)
{
    void* ptr;
    uint32_t real_res_num = 0;
    if (ubf == NULL || k > ubf_get_stock(ubf)-1)//缓存器不存在 或 读取超过容量
        return 0;
    for (uint32_t cnt = 0; cnt < num; ++cnt)//cnt∈[0, num-1]，共（num-1-0+1=num）次自然循环
    {
        ptr = ubf_pop(ubf, k + cnt);
        if (ptr)
        {
            memcpy((uint8_t*)pdata+cnt*ubf->size, ptr, ubf->size);
            ++real_res_num;
        }
        else break;
    }
    return real_res_num;
}

/* 首元素为最旧的数据 */
uint32_t ubf_pop_into_array_old2new(ubf_t ubf, void* pdata, uint32_t k, uint32_t num)
{
    void* ptr;
    uint32_t real_num = 0;
    if (ubf == NULL || k > ubf_get_stock(ubf)-1)//缓存器不存在 或 读取超过容量
        return 0;
    for (uint32_t cnt = k+num-1; cnt+1 > k; --cnt)
    {
        ptr = ubf_pop(ubf, cnt);
        if (ptr)
        {
            memcpy((uint8_t*)pdata+real_num*ubf->size, ptr, ubf->size);
            ++real_num;
        }
    }
    return real_num;
}

/**
 * @brief 模块测试代码,解注释后，自行查看内存值验证
 */
// #define TEST_ENABLE

#ifdef TEST_ENABLE
#include "stdio.h"

int main()
{
    uint32_t i;
    /* 例程1：声明缓存区、存取数据、删除缓存区 */
    struct _test_t
    {
        float a;
        int b;
        char c;
    }test;
    struct _test_t* tt;
    test.a = 0.1;
    test.b = 0;
    test.c = 124;

    ubf_t ubf1 = ubf_create(5, sizeof(struct _test_t));// 创建唤醒缓存器指针

    for (i=0; i<8; i++)
    {
        ubf_push(ubf1, &test);// 存入数据
        test.b++;
    }
    for (i=0; i<8; i++)
    {
        tt = ubf_pop(ubf1, i);// 提取第K-i时刻的数据
    }
    ubf_delete(ubf1);// 释放缓存区


    /* 例程2：提取数据进一维数组 */
    uint32_t real_res, array_new[20]={}, array_old[20]={},
        num_indeed = 15, num_inneed = 21;//num_indeed缓存器大小（≤20），num_inneed需要取出的数据
    ubf_t ubf = ubf_create(num_indeed, sizeof(uint32_t));// 创建唤醒缓存器指针
    if (ubf)
    {
        for (i=0; i<num_indeed; ++i)
        {
            ubf_push(ubf, &i);
        }
        real_res = ubf_pop_into_array_new2old(ubf, array_new, 4, num_inneed);
        printf("the res new array is as follow:\n");
        for (i=0; i<20; ++i)
        {
            printf("%d ", array_new[i]);
        }
        printf("\nthe real num of the res = %d", real_res);

        printf("\n-------------------------------------\n");

        real_res = ubf_pop_into_array_old2new(ubf, array_old, 4, num_inneed);
        printf("the res old array is as follow:\n");
        for (i=0; i<20; ++i)
        {
            printf("%d ", array_old[i]);
        }
        printf("\nthe real num of the res = %d", real_res);

        ubf_delete(ubf);// 释放缓存区
    }
    return 0;
}
#endif

