#pragma once


template <class T, size_t memory_size, size_t max_data_size>
class CircularBuffer
{
private:
    uint8_t buffer[memory_size];
    uint8_t num_elements = 0;
    uint16_t free_mem = memory_size;
    uint16_t readptr=0, writeptr=0;
public:
    // Get a pointer to write to
    T* reserve()
    {
        size_t elem_size = sizeof(T)+max_data_size;
        if (writeptr+elem_size > memory_size)
        {
            return reinterpret_cast<T*>(buffer);
        }
        return reinterpret_cast<T*>(&buffer[writeptr]);
    }

    // Advance write pointer (after reserve)
    void write(size_t data_size)
    {
        size_t elem_size_initial = sizeof(T)+max_data_size;
        if (writeptr+elem_size_initial > memory_size)
        {
            writeptr = 0;
        }
        size_t elem_size = sizeof(T) + data_size;
        writeptr += elem_size;
        num_elements++;
        free_mem -= elem_size;
    }

    // Read data at the read pointer location
    T* peek()
    {
        return reinterpret_cast<T*>(&buffer[readptr]);
    }

    // Advance read pointer (after peek)
    void read(size_t data_size)
    {
        size_t elem_size = sizeof(T)+data_size;
        readptr += elem_size;
        free_mem += elem_size;
        if (readptr + sizeof(T)+max_data_size > memory_size)
        {
            readptr = 0;
        }
        num_elements--;
        return;
    }

    uint8_t get_num_elements() { return num_elements; }

    uint16_t get_free() { return free_mem; }
};

#if defined(_STM32_DEF_)

//Single-threaded, don't care about mutex
#define SafeCircularBuffer CircularBuffer

#endif

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32)

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

template <class T>
class SafeCircularBuffer
{
    public:
        SafeQueue(int Size=16) 
        {
            Queue = xQueueCreate( Size, sizeof( T ) );
            NumElements = 0;
            QueueSema = xSemaphoreCreateCounting( 1, 0 );        
        }

        // Add an element to the queue.
        // ISR, absolutely no printing to serial!!
        void Enqueue(T t)
        {
            xSemaphoreTakeFromISR(QueueSema, 0);
            xQueueSendFromISR(Queue, &t, 0);
            NumElements++;
            xSemaphoreGiveFromISR(QueueSema, 0);
        }

        // Get the "front"-element.
        // If the queue is empty, wait till an element is avaiable.
        // ISR, absolutely no printing to serial!!
        bool Dequeue(T* pVal)
        {
            xSemaphoreTakeFromISR(QueueSema, 0);
            if(xQueueReceiveFromISR(Queue, pVal, 0))
            {
                NumElements--;
                xSemaphoreGiveFromISR(QueueSema, 0);
                return true;
            }
            xSemaphoreGiveFromISR(QueueSema, 0);
            return false;
        }

        int NumElements;

private:
  QueueHandle_t Queue;
  SemaphoreHandle_t QueueSema;
};

#endif
