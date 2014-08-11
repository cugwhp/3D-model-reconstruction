#ifdef _WIN32
#include <windows.h>
#endif

#include <stdlib.h>
#include <vector>

#include <gl/GL.h>			// Header File For The OpenGL32 Library
#include <gl/GLU.h>			// Header File For The GLu32 Library
#include <GL/glut.h>

#include "drawing.h"

std::vector<GLuint> suggest_direction_draw;

void draw_arrow() {
	//done
	GLuint donelist = glGenLists(1);
	glNewList(donelist, GL_COMPILE);
	glBegin(GL_QUADS);
	glVertex2f(0, 1);
	glVertex2f(1, 2);
	glVertex2f(2, 1);
	glVertex2f(1, 0);
	glEnd();
	glEndList();
	suggest_direction_draw.push_back(donelist);

	//up
	GLuint uplist = glGenLists(1);
	glNewList(uplist, GL_COMPILE);
	glBegin(GL_LINE_STRIP);
	glVertex2f(0, 0);
	glVertex2f(1, 2);
	glVertex2f(2, 0);
	glEnd();
	glEndList();
	suggest_direction_draw.push_back(uplist);

	//right
	GLuint rightlist = glGenLists(1);
	glNewList(rightlist, GL_COMPILE);
	glBegin(GL_LINE_STRIP);
	glVertex2f(0, 2);
	glVertex2f(2, 1);
	glVertex2f(0, 0);
	glEnd();
	glEndList();
	suggest_direction_draw.push_back(rightlist);

	//down
	GLuint downlist = glGenLists(1);
	glNewList(downlist, GL_COMPILE);
	glBegin(GL_LINE_STRIP);
	glVertex2f(0, 2);
	glVertex2f(1, 0);
	glVertex2f(2, 2);
	glEnd();
	glEndList();
	suggest_direction_draw.push_back(downlist);

	//left
	GLuint leftlist = glGenLists(1);
	glNewList(leftlist, GL_COMPILE);
	glBegin(GL_LINE_STRIP);
	glVertex2f(2, 2);
	glVertex2f(0, 1);
	glVertex2f(2, 0);
	glEnd();
	glEndList();
	suggest_direction_draw.push_back(leftlist);
}

GLuint* get_arrow(int direction) {
	if (suggest_direction_draw.empty()) draw_arrow();

	return &suggest_direction_draw[direction];
}

