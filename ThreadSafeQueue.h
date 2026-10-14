#pragma once


template <class T, size_t memory_size, size_t max_data_size, bool rx>
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

#include "freertos/ringbuf.h"
#include <type_traits>
#include <array>

template <class T, size_t memory_size, size_t max_data_size, bool rx>
class SafeCircularBuffer
{
    RingbufHandle_t buffer_handle;
    std::array<uint8_t, (sizeof(T) + max_data_size) * rx> write_data;
    T* read_item_pointer;
    
    
public:
    SafeCircularBuffer()
    {
        if(rx)
        {
            buffer_handle = xRingbufferCreateNoSplit(sizeof(T) + max_data_size, memory_size/(sizeof(T) + max_data_size));
            read_item_pointer = nullptr;
        }
    }

    T* IRAM_ATTR reserve()
    {
        if (rx)
        {
            return reinterpret_cast<T*>(write_data.data());
        }
        else
        {
            return nullptr;
        }
        
    }

    bool IRAM_ATTR write(size_t data_size)
    {
        if (rx)
        {
            BaseType_t awoken = pdFALSE;
            xRingbufferSendFromISR(buffer_handle, write_data.data(), sizeof(T)+ data_size, &awoken);
            return awoken == pdTRUE;
        }
        else
        {
            return false;
        }
        
    }

    T* IRAM_ATTR peek()
    {
        if (rx)
        {
            size_t item_size;
            read_item_pointer = reinterpret_cast<T*>(xRingbufferReceive(buffer_handle, &item_size, 0));
            return read_item_pointer;
        }
        else
        {
            return nullptr;
        }
        
    }

    bool IRAM_ATTR read(size_t data_size)
    {
        if (rx)
        {
            vRingbufferReturnItem(buffer_handle, read_item_pointer);
            return false;
        }
        else
        {
            return false;
        }
        
    }

    uint8_t get_num_elements() 
    { 
        if (rx)
        {
            UBaseType_t waiting;
            vRingbufferGetInfo(buffer_handle, nullptr, nullptr, nullptr, nullptr, &waiting); 
            return waiting;
        }
        else
        {
            return 0;
        }
        
        
    }

    uint16_t get_free() 
    { 
        if (rx)
        {
            return xRingbufferGetCurFreeSize(buffer_handle);
        }
        else
        {
            return 0;
        }
        
    }
};

#endif
