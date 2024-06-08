// Copyright 2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
    typedef struct
    {
        float mean;
        int _sum;
        size_t _ind;
        size_t _len;
        int *_values;
    } MyMeanFilter;

    void MyMeanFilterInit(MyMeanFilter *f, int buf[], size_t len);

    float MyMeanFilterUpdate(MyMeanFilter *f, int value);

#ifdef __cplusplus
}
#endif
