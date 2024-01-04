#pragma once

#include <sys/un.h>

struct MemoryWrapper
{
public:
    MemoryWrapper(void *mem, size_t size)
        : size(size)
    {
        this->mem = malloc(size);
        if (this->mem == nullptr)
        {
            throw std::bad_alloc();
        }
        memcpy(this->mem, mem, size);
    }

    ~MemoryWrapper()
    {
        free(this->mem);
    }

    MemoryWrapper(MemoryWrapper &&other)
    {
        this->operator=(std::move(other));
    }

    MemoryWrapper &operator=(MemoryWrapper &&other)
    {
        this->mem = other.mem;
        this->size = other.size;

        other.mem = nullptr;
        return *this;
    }

    MemoryWrapper(const MemoryWrapper &) = delete;
    MemoryWrapper &operator=(const MemoryWrapper &) = delete;

    void *mem;
    size_t size;
};