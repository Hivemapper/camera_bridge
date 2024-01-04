#pragma once

#include <sys/un.h>

struct MemoryWrapper
{
public:
    MemoryWrapper(void *mem, size_t memSize, void *exifMem, size_t exifMemSize)
        : memSize(memSize), exifMemSize(exifMemSize)
    {
        this->mem = malloc(memSize);
        if (this->mem == nullptr)
        {
            throw std::bad_alloc();
        }
        this->exifMem = malloc(exifMemSize);
        if (this->exifMem == nullptr)
        {
            this->~MemoryWrapper();
            throw std::bad_alloc();
        }
        memcpy(this->mem, mem, memSize);
        memcpy(this->exifMem, exifMem, exifMemSize);
    }
    ~MemoryWrapper()
    {
        free(this->mem);
        free(this->exifMem);
    }

    MemoryWrapper(MemoryWrapper &&other)
    {
        this->operator=(std::move(other));
    }

    MemoryWrapper &operator=(MemoryWrapper &&other)
    {
        this->mem = other.mem;
        this->memSize = other.memSize;
        this->exifMem = other.exifMem;
        this->exifMemSize = other.exifMemSize;

        other.mem = nullptr;
        other.exifMem = nullptr;
        return *this;
    }

    MemoryWrapper(const MemoryWrapper &) = delete;
    MemoryWrapper &operator=(const MemoryWrapper &) = delete;

    void *mem;
    size_t memSize;
    void *exifMem;
    size_t exifMemSize;
};