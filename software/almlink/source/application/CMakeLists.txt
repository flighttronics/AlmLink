# Make sure the linker can find
link_directories (${ALM3LINK_BINARY_DIR}/drivers)
link_directories (${ALM3LINK_BINARY_DIR}/drivers/spi)
link_directories (${ALM3LINK_BINARY_DIR}/common) 
link_directories (${ALM3LINK_BINARY_DIR}/common/rfm23bp) 
link_directories (${ALM3LINK_BINARY_DIR}/libraries/rscode) 

# Add executable called "helloDemo" that is built from the source files 
# "demo.cxx" and "demo_b.cxx". The extensions are automatically found. 
add_executable (almlink main.c) 

# Link the executable to the Hello library. 
target_link_libraries (almlink rfm23bp rfm23bp_lookup gpio spi m) 
