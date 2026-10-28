#pragma once


template <class T, size_t memory_size, size_t max_data_size, bool rx>
class CircularBuffer
{
private:
    unsigned int buffer[memory_size/sizeof(unsigned int)];
    uint8_t num_elements = 0;
    uint16_t free_mem = memory_size/sizeof(unsigned int);
    uint16_t readptr=0, writeptr=0;
public:


    constexpr size_t get_num_slots(size_t data_size) 
    {
        return (sizeof(T)+data_size + sizeof(unsigned int) -1) / sizeof(unsigned int);
    }

    constexpr size_t get_max_slots() 
    {
        return get_num_slots(max_data_size);
    }

    // Get a pointer to write to
    T* reserve()
    {
        if (writeptr+get_max_slots() > memory_size / sizeof(unsigned int))
        {
            return reinterpret_cast<T*>(buffer);
        }
        return reinterpret_cast<T*>(&buffer[writeptr]);
    }

    // Advance write pointer (after reserve)
    void write(size_t data_size)
    {
        if (writeptr+get_max_slots() > memory_size / sizeof(unsigned int))
        {
            free_mem -= (memory_size / sizeof(unsigned int)) - writeptr;
            writeptr = 0;
        }
        size_t elem_size = get_num_slots(data_size);
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
        size_t elem_size = get_num_slots(data_size);
        readptr += elem_size;
        free_mem += elem_size;
        if (readptr + get_max_slots() > memory_size / sizeof(unsigned int))
        {
            free_mem += (memory_size / sizeof(unsigned int)) - readptr;
            readptr = 0;
        }
        num_elements--;
        return;
    }

    uint8_t get_num_elements() { return num_elements; }

    uint16_t get_free() { return free_mem * sizeof(unsigned int); }
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
    RingbufHandle_t buffer_handle = nullptr;
    std::array<uint8_t, (sizeof(T) + max_data_size) * rx> write_data;
    T* read_item_pointer = nullptr;
    
public:
    SafeCircularBuffer()
    {
        if constexpr (rx)
        {
            buffer_handle = xRingbufferCreateNoSplit(sizeof(T) + max_data_size, memory_size/(sizeof(T) + max_data_size));
            read_item_pointer = nullptr;
        }
    }

    T* IRAM_ATTR reserve()
    {
        if constexpr (rx)
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
        if constexpr (rx)
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

    T* peek()
    {
        if constexpr (rx)
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

    bool read(size_t data_size)
    {
        if constexpr (rx)
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
        if constexpr (rx)
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
        if constexpr (rx)
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
