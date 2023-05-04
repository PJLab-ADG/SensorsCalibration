function(add_example basename)
  # set C++ standard to C++17
  set(CMAKE_CXX_STANDARD 17)

  # add WITH_EIGEN, if specified
  if(WITH_EIGEN)
    add_definitions("-DWITH_EIGEN")
  endif()

  # add the exectuable and link it to the Python libs
  add_executable(${basename} ${basename}.cpp ../matplotlibcpp.h)
  target_link_libraries(${basename} ${Python3_LIBRARIES})
endfunction(add_example)

# add the executables
add_example(axlines)
add_example(bar)
add_example(basic)
add_example(contour)
add_example(eigen)
add_example(errorbar)
add_example(fill_inbetween)
add_example(fill)
add_example(imshow)
add_example(legend)
add_example(loglog)
add_example(minimal)
add_example(modern)
add_example(nonblock)
add_example(quiver)
add_example(scatter)
add_example(semilogx)
add_example(semilogy)
add_example(small)
add_example(spy)
add_example(subplot)
add_example(surface)
add_example(update)
add_example(xkcd)
