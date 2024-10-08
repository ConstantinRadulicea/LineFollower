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

#ifndef __MEDIANFILTER_H__
#define __MEDIANFILTER_H__

//#include <stdio.h>


#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

class MedianFilter
{
private:
    unsigned int _windowSize;
    int totSamples;
    double* _queue;
    double* _tempQueue;


    // An optimized version of Bubble Sort
    static void bubbleSort(double arr[], int n)
    {
        int i, j;
        int swapped;
        double temp;
        for (i = 0; i < n - 1; i++) {
            swapped = 0;
            for (j = 0; j < n - i - 1; j++) {
                if (arr[j] > arr[j + 1]) {
                    //swap(arr[j], arr[j + 1]);
                    temp = arr[j];
                    arr[j] = arr[j + 1];
                    arr[j + 1] = temp;
                    swapped = 1;
                }
            }

            // If no two elements were swapped
            // by inner loop, then break
            if (swapped == 0)
                break;
        }
    }
public:

    MedianFilter(unsigned int windowSize) {
        this->totSamples = 0;
        this->_windowSize = windowSize;
        this->_queue = new double[windowSize];
        this->_tempQueue = new double[windowSize];
        for (int i = 0; i < windowSize; i++) {
            this->_queue[i] = 0.0f;
        }
    }

    double next(double curValue) {
        double nextValue;
        this->totSamples++;
        this->totSamples = MIN(this->totSamples, this->_windowSize);


        for (int i = 0; i < (this->_windowSize -1); i++) {
            this->_queue[i] = this->_queue[i+1];
        }
        this->_queue[this->_windowSize-1] = curValue;
        //for (int i = 0; i < this->_windowSize; i++) {
        //    printf("%f\n", this->_queue[i]);
        //}
        //printf("\n");


        for (int i = 0; i < this->_windowSize; i++) {
            _tempQueue[i] = _queue[i];
        }
        this->bubbleSort(this->_tempQueue, this->_windowSize);
        nextValue = this->_tempQueue[(int)((this->_windowSize-1) - (this->totSamples / 2))];


        //for (int i = 0; i < this->_windowSize; i++) {
        //    printf("%f\n", this->_tempQueue[i]);
        //}
        //printf("\n");
        return nextValue;
    }

    double nextVolatile(double curValue) volatile {
        double nextValue;
        this->totSamples++;
        this->totSamples = MIN(this->totSamples, this->_windowSize);


        for (int i = 0; i < (this->_windowSize - 1); i++) {
            this->_queue[i] = this->_queue[i + 1];
        }
        this->_queue[this->_windowSize - 1] = curValue;
        //for (int i = 0; i < this->_windowSize; i++) {
        //    printf("%f\n", this->_queue[i]);
        //}
        //printf("\n");


        for (int i = 0; i < this->_windowSize; i++) {
            _tempQueue[i] = _queue[i];
        }
        this->bubbleSort(this->_tempQueue, this->_windowSize);
        nextValue = this->_tempQueue[(int)((this->_windowSize - 1) - (this->totSamples / 2))];


        //for (int i = 0; i < this->_windowSize; i++) {
        //    printf("%f\n", this->_tempQueue[i]);
        //}
        //printf("\n");
        return nextValue;
    }

    MedianFilter() {
        this->totSamples = 0;
        this->_windowSize = 0;
    }

    ~MedianFilter() {
        delete this->_queue;
        delete this->_tempQueue;
    }
};

#endif