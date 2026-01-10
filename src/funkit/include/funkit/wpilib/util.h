#include <algorithm>
#include <vector>

namespace funkit::wpilib {

#define vector_has(vec, val) std::find(vec.begin(), vec.end(), val) != vec.end()

}  // namespace funkit::wpilib