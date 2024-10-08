/*
* Copyright 2024 Constantin Dumitru Petre RĂDULICEA
* 
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* 
*   http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#ifndef __MOVING_AVERAGE_H__
#define __MOVING_AVERAGE_H__
#include <stddef.h>

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

class MovingAverage {
  size_t size, head = 0, count = 0;
  float windowSum = 0.0f;
  float* queue;

public:
MovingAverage(){
  this->size = 0;
  this->head = 0;
  this->count = 0;
  this->windowSum = 0.0f;
  this->queue = NULL;
}
MovingAverage(size_t size) {
    this->size = size;
    queue = new float[size];
    for (size_t i = 0; i < size; i++) {
      queue[i] = 0.0f;
    }
    
  }

  MovingAverage() {
    this->size = 0;
  }

float next(float val){
    ++count;
    // calculate the new sum by shifting the window
    size_t tail = (head + 1) % size;
    windowSum = windowSum - queue[tail] + val;
    // move on to the next head
    head = (head + 1) % size;
    queue[head] = val;
    return ((float)windowSum) / ((float)MIN(size, count));
  }

  float nextVolatile(float val) volatile {
      ++count;
      // calculate the new sum by shifting the window
      size_t tail = (head + 1) % size;
      windowSum = windowSum - queue[tail] + val;
      // move on to the next head
      head = (head + 1) % size;
      queue[head] = val;
      return ((float)windowSum) / ((float)MIN(size, count));
    }

  ~MovingAverage(){
    delete this->queue;
  }
};


#endif