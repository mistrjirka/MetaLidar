// Include guard
#ifndef SHARED_MEMORY_H // Check if SHARED_MEMORY_H is not defined
#define SHARED_MEMORY_H // Define SHARED_MEMORY_H

#include <iostream>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#define SHM_SIZE 4096

class SharedMemory {
    private:
        int fd;
        void *ptr;
        size_t size;
    public:
        SharedMemory(const char *name, size_t size);
        ~SharedMemory();
        void *get_ptr();
        void resize(size_t new_size); // Resize function
};

#endif // End of include guard
