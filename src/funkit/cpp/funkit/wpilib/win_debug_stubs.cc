#if defined(_WIN32) && defined(_DEBUG)

#include <algorithm>
#include <cstddef>

extern "C" const char* __cdecl __std_find_end_1(const char* _First1,
                                                const char* _Last1,
                                                const char* _First2,
                                                std::size_t _Count2) {
  return std::find_end(_First1, _Last1, _First2, _First2 + _Count2);
}

#endif
