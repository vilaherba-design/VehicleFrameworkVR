#include "telemetry/CsvLog.h"

namespace vfvr
{
    void CsvLog::open(const std::string& path, const std::string& header, const std::size_t maxRows)
    {
        std::scoped_lock lock(_mutex);
        if (_open) {
            return;
        }
        _file.open(path, std::ios::out | std::ios::trunc);
        if (!_file) {
            logger::warn("Telemetry: cannot write '{}'", path);
            return;
        }
        _file << header << '\n';
        _path = path;
        _rows = 0;
        _maxRows = maxRows;
        _open = true;
        _capReported = false;
        logger::info("Telemetry: writing '{}'", path);
    }

    void CsvLog::close()
    {
        flush();
        std::scoped_lock lock(_mutex);
        if (_open) {
            _file.close();
            _open = false;
        }
        _pending.clear();
    }

    void CsvLog::add(std::string row)
    {
        std::scoped_lock lock(_mutex);
        if (!_open) {
            return;
        }
        if (_rows + _pending.size() >= _maxRows) {
            if (!_capReported) {
                _capReported = true;
                logger::info("Telemetry: '{}' reached its row cap ({}), no more rows this session", _path, _maxRows);
            }
            return;
        }
        _pending.push_back(std::move(row));
    }

    void CsvLog::flush()
    {
        std::vector<std::string> rows;
        {
            std::scoped_lock lock(_mutex);
            if (!_open || _pending.empty()) {
                return;
            }
            rows.swap(_pending);
            _rows += rows.size();
        }
        for (const auto& row : rows) {
            _file << row << '\n';
        }
        _file.flush();
    }
}
