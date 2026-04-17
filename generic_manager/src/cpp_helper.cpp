#include "cpp_helper.hpp"

CppHelper::CppHelper()
{}

bool CppHelper::convert_string_to_bool(std::string string)
{
    for (auto &x : string)
    {
        x = tolower(x);
    }
    
    bool b;
    std::istringstream(string) >> std::boolalpha >> b;
    return b;
}

