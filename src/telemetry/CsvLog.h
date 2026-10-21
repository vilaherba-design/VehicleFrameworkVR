#pragma once

#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace vfvr
{
    /**
     * Buffered CSV file: rows can be added from any thread (the physics step adds from Havok's thread),
     * and the game thread writes them out in blocks with flush(). Stops adding rows past a row cap so a
     * long session cannot fill the disk.
     */
    class CsvLog
    {
    public:
        void open(const std::string& path, const std::string& header, std::size_t maxRows);
        void close();

        bool isOpen() const
        {
            return _open;
        }

        void add(std::string row);
        void flush();

    private:
        std::mutex _mutex;
        std::vector<std::string> _pending;
        std::ofstream _file;
        std::string _path;
        std::size_t _rows = 0;
        std::size_t _maxRows = 0;
        bool _open = false;
        bool _capReported = false;
    };
}
