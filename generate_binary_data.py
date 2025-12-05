#!/usr/bin/env python3
"""
Generate BinaryData.h and BinaryData.cpp from knob.png
"""
import os
import sys

def generate_binary_data():
    # Read the PNG file
    png_path = "Resources/knob.png"

    if not os.path.exists(png_path):
        print(f"Error: {png_path} not found!")
        print("Please place knob.png in the Resources folder first.")
        sys.exit(1)

    with open(png_path, 'rb') as f:
        png_data = f.read()

    data_size = len(png_data)

    # Generate BinaryData.h
    header_content = """// BinaryData.h
// Auto-generated from knob.png

#pragma once

namespace BinaryData
{
    extern const char*  knob_png;
    const int           knob_pngSize = """ + str(data_size) + """;
}
"""

    # Generate BinaryData.cpp
    cpp_content = """// BinaryData.cpp
// Auto-generated from knob.png

#include "BinaryData.h"

static const unsigned char knob_png_data[] = {
"""

    # Convert binary data to C array
    for i, byte in enumerate(png_data):
        if i % 16 == 0:
            cpp_content += "\n    "
        cpp_content += f"0x{byte:02x},"

    cpp_content = cpp_content.rstrip(',')
    cpp_content += """
};

const char* BinaryData::knob_png = (const char*) knob_png_data;
"""

    # Write files
    with open("BinaryData.h", 'w') as f:
        f.write(header_content)

    with open("BinaryData.cpp", 'w') as f:
        f.write(cpp_content)

    print(f"✅ Generated BinaryData.h and BinaryData.cpp")
    print(f"   Image size: {data_size} bytes")

if __name__ == "__main__":
    generate_binary_data()
