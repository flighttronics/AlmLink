# Make sure the compiler can find include files from our libraries 
include_directories(rscode)
include_directories(fifo)
  

#Add Subdirectories
add_subdirectory(rscode)
add_subdirectory(fifo)

