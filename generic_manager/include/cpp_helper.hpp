#include <string>
#include <array>
#include <charconv>
#include <stdexcept>
#include <sstream>

#pragma once

class CppHelper {
    public:
        CppHelper();

        bool convert_string_to_bool(std::string string);

        template <typename T>
        std::string convert_to_string(T value)
        {
            using U = std::remove_cv_t<std::remove_reference_t<T>>;

            static_assert(
                std::is_same_v<U, int> ||
                std::is_same_v<U, float> ||
                std::is_same_v<U, double>,
                "T must be int, float, or double"
            );

            // Fast Path:: integers
            if constexpr (std::is_same_v<U, int>)
            {
                std::array<char, 64> buffer; //make sure buffer is big enough to support large values

                auto result = std::to_chars(buffer.data(),
                                            buffer.data() + buffer.size(),
                                            value);

                if (result.ec != std::errc())
                {
                    throw std::runtime_error("Integer conversion failed");
                }

                return std::string(buffer.data(), result.ptr);
            }
            // Fallback: floats/doubles
            else
            {
                return std::to_string(value);
            }
        }
};
