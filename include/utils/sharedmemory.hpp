#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <system_error>
#include <string>

template <typename T>
class SharedMemory
{
public:
    SharedMemory(const std::string &name)
        : _name("/" + name), _fd(-1), _data(nullptr)
    {
        _size = sizeof(T);
        _fd = shm_open(_name.c_str(), O_CREAT | O_RDWR, 0666);
        if (_fd == -1)
        {
            throw std::system_error(errno, std::generic_category(), "Failed to open shared memory");
        }

        if (ftruncate(_fd, _size) == -1)
        {
            throw std::system_error(errno, std::generic_category(), "Failed to set size of shared memory");
        }
        _data = mmap(nullptr, _size, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, 0);
        if (_data == MAP_FAILED)
        {
            throw std::system_error(errno, std::generic_category(), "Failed to map shared memory");
        }
    }

    ~SharedMemory()
    {
        if (_data)
        {
            munmap(_data, _size);
        }
        if (_fd != -1)
        {
            close(_fd);
        }
        shm_unlink(_name.c_str());
    }

    T *data() const
    {
        return static_cast<T *>(_data);
    }

private:
    std::string _name; // Name of the shared memory
    std::size_t _size; // Size of the shared memory
    int _fd;           // File descriptor for the shared memory
    void *_data;       // Mapped memory address
};
