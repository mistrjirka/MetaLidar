#include "SharedMemory/SharedMemory.h"

SharedMemory::SharedMemory(const char *name, size_t size) {
    // Open a shared memory object
    fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (fd == -1) {
        perror("shm_open");
        exit(1);
    }

    // Resize the shared memory object
    if (ftruncate(fd, size) == -1) {
        perror("ftruncate");
        exit(1);
    }

    // Map the shared memory object
    ptr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
        perror("mmap");
        exit(1);
    }

    // Store the size
    this->size = size;
}

SharedMemory::~SharedMemory() {
    // Unmap the shared memory object
    if (munmap(ptr, size) == -1) {
        perror("munmap");
        exit(1);
    }

    // Close the shared memory object
    if (close(fd) == -1) {
        perror("close");
        exit(1);
    }

    // Optionally, unlink the shared memory object
    // if (shm_unlink(SHM_NAME) == -1) {
    //     perror("shm_unlink");
    //     exit(1);
    // }
}

void *SharedMemory::get_ptr() {
    // Return the pointer to the shared memory
    return ptr;
}

void SharedMemory::resize(size_t new_size) {
    // Resize function
    // Check if the new size is valid and different
    if (new_size <= 0 || new_size == size) {
        std::cout << "Invalid or same size\n";
        return;
    }

    // Resize the shared memory object
    if (ftruncate(fd, new_size) == -1) {
        perror("ftruncate");
        exit(1);
    }

    // Unmap the old memory mapping
    if (munmap(ptr, size) == -1) {
        perror("munmap");
        exit(1);
    }

    // Map the new memory mapping
    ptr = mmap(NULL, new_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
        perror("mmap");
        exit(1);
    }

    // Update the size
    size = new_size;
}

