# Make sure the compiler can find include files from our libraries 
include_directories (application)  
include_directories (drivers)  
include_directories (drivers/spi) 
include_directories (drivers/gpio) 
include_directories (common)  
include_directories (common/rfm23bp)  
include_directories (libraries)  
include_directories (libraries/rscode)
include_directories (libraries/fifo)

#Add Subdirectories
add_subdirectory(application)
add_subdirectory(drivers)
add_subdirectory(common)
add_subdirectory(libraries)
