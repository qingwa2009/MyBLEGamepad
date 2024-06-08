#include <stdio.h>
#include <string.h>
#include "MyMeanFilter.h"

void MyMeanFilterInit(MyMeanFilter *f, int buf[], size_t len)
{
    f->mean = 0;
    f->_sum = 0;
    f->_ind = 0;
    f->_len = len / sizeof(buf[0]);
    f->_values = buf;
    memset(buf, 0, len);
}

float MyMeanFilterUpdate(MyMeanFilter *f, int value)
{
    f->_ind++;
    if (f->_ind >= f->_len)
        f->_ind = 0;

    int oldValue = f->_values[f->_ind];
    f->_values[f->_ind] = value;

    f->_sum += (value - oldValue);
    f->mean = f->_sum / (float)f->_len;
    return f->mean;
}