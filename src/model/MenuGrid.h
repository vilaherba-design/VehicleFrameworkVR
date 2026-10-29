#pragma once

#include <algorithm>

namespace vfvr::model
{
    // The vehicle and wheel menus as a grid: two columns of twelve, filled down the first column and then the second (so the list reads
    // in order), a further page for every 24 more entries.
    struct MenuGrid
    {
        static constexpr int COLUMNS = 2;
        static constexpr int ROWS = 12;
        static constexpr int PER_PAGE = COLUMNS * ROWS;

        static int pages(const int count) { return (std::max)(1, (count + PER_PAGE - 1) / PER_PAGE); }

        // Entries shown on a page (0 past the last page).
        static int countOn(const int count, const int page) { return std::clamp(count - page * PER_PAGE, 0, PER_PAGE); }

        // The entry in a cell of a page, or -1 for an empty cell.
        static int entryAt(const int count, const int page, const int column, const int row)
        {
            if (column < 0 || column >= COLUMNS || row < 0 || row >= ROWS) {
                return -1;
            }
            const int i = page * PER_PAGE + column * ROWS + row;
            return i < count ? i : -1;
        }

        // Rows the page's columns need (the panel is no taller than its longest column).
        static int rowsOn(const int count, const int page) { return (std::min)(ROWS, countOn(count, page)); }
    };
}
