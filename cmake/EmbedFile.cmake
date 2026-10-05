# Script mode helper for EmbedResource.cmake: converts INPUT into a C++ byte array.
file(READ ${INPUT} hex HEX)
string(LENGTH "${hex}" hex_len)
math(EXPR size "${hex_len} / 2")
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," bytes "${hex}")
# Break into lines of 32 bytes to keep compilers and editors happy.
string(REGEX REPLACE "((0x[0-9a-f][0-9a-f],){32})" "\\1\n" bytes "${bytes}")
file(WRITE ${OUTPUT}
"// Generated from ${INPUT} - do not edit.
#include <cstddef>
namespace ${NS} {
extern const unsigned char ${IDENT}_data[];
extern const std::size_t ${IDENT}_size;
alignas(16) const unsigned char ${IDENT}_data[] = {
${bytes}
};
const std::size_t ${IDENT}_size = ${size};
} // namespace ${NS}
")
