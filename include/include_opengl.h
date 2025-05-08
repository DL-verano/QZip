#pragma once
#ifdef __APPLE__
//#include <OpenGL/gl.h>
#include <OpenGL/glew.h>
#include <OpenGL/glut.h>
#else
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#undef NOMINMAX
#endif
//#include <GL/gl.h>
//#define GLEW_STATIC
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/freeglut.h>
#endif
