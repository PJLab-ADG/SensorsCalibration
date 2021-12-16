/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Liu Zhuochun <liuzhuochun@pjlab.org.cn>
 */
#include "aruco_marker/simplifyPath.hpp"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define _CRT_SECURE_NO_WARNINGS
#define HOLE_BORDER 1
#define OUTER_BORDER 2

int **create2dArray(int r, int c) {
  int **arr;
  if ((arr = (int **)malloc(sizeof(int *) * r)) == NULL)
    perror("malloc failed");

  for (int i = 0; i < r; i++) {
    if ((arr[i] = (int *)malloc(sizeof(int) * c)) == NULL)
      perror("malloc failed");
  }

  return arr;
}

void free2dArray(int **arr, int r) {
  for (int i = 0; i < r; i++) {
    free(arr[i]);
  }
  free(arr);
}

struct Border {
  int seq_num;
  int border_type;
};

bool samePoint(struct PointRDP a, struct PointRDP b) {
  return a.y == b.y && a.x == b.x;
}

void setPoint(struct PointRDP *p, int r, int c) {
  assert(p);
  p->x = c;
  p->y = r;
}

struct Node {
  struct Border border;
  int parent;
  int first_child;
  int next_sibling;
};

void setNode(struct Node *n, int p, int fc, int ns) {
  assert(n);
  n->parent = p;
  n->first_child = fc;
  n->next_sibling = ns;
}

void resetNode(struct Node *n) {
  assert(n);
  n->parent = -1;
  n->first_child = -1;
  n->next_sibling = -1;
}

struct nodeVector {
  struct Node *vector;
  int current_max;
  int current_index;
};

void initNodeVector(struct nodeVector *nv) {
  nv->current_max = 10;
  nv->current_index = 0;
  nv->vector = (struct Node *)malloc(sizeof(struct Node) * (nv->current_max));
}

void resizeNodeVector(struct nodeVector *nv) {
  struct Node *tmp;
  if ((tmp = (struct Node *)realloc(
           nv->vector, sizeof(struct Node) * (nv->current_max * 2))) == NULL) {
    free(nv->vector);
    perror("malloc failed");
  }
  nv->current_max *= 2;
  nv->vector = tmp;
}

void addNodeVector(struct nodeVector *nv, struct Node node) {
  if (nv->current_index + 1 >= nv->current_max)
    resizeNodeVector(nv);
  nv->vector[nv->current_index] = node;
  nv->current_index += 1;
}

struct Node *trimNodeVector(struct nodeVector *nv, int *vector_size) {
  struct Node *tmp;
  if ((tmp = (struct Node *)realloc(
           nv->vector, sizeof(struct Node) * (nv->current_index))) == NULL) {
    free(nv->vector);
    perror("malloc failed");
  }
  (*vector_size) = nv->current_index;
  return tmp;
}

struct intVector {
  int *vector;
  int current_max;
  int current_index;
};

void initIntVector(struct intVector *iv) {
  iv->current_index = 0;
  iv->current_max = 10;
  iv->vector = (int *)malloc(sizeof(int) * (iv->current_max));
}

void resizeIntVector(struct intVector *iv) {
  int *tmp;
  if ((tmp = (int *)realloc(iv->vector, sizeof(int) * (iv->current_max * 2))) ==
      NULL) {
    free(iv->vector);
    perror("malloc failed");
  }
  iv->current_max *= 2;
  iv->vector = tmp;
}

void addIntVector(struct intVector *iv, int value) {

  if (iv->current_index + 1 >= iv->current_max)
    resizeIntVector(iv);
  iv->vector[iv->current_index] = value;
  iv->current_index += 1;
}

int *trimIntVector(struct intVector *iv, int *vector_size) {
  int *tmp;
  if ((tmp = (int *)realloc(iv->vector, sizeof(int) * (iv->current_index))) ==
      NULL) {
    free(iv->vector);
    perror("malloc failed");
  }
  (*vector_size) = iv->current_index;
  return tmp;
}

struct Pixel {
  unsigned char red;
  unsigned char blue;
  unsigned char green;
};

void setPixel(struct Pixel *p, unsigned char r, unsigned char g,
              unsigned char b) {
  p->red = r;
  p->green = g;
  p->blue = b;
}

struct pointVector {
  struct PointRDP *vector;
  int current_max;
  int current_index;
};

struct point2dVector {
  struct PointRDP **vector;
  int current_max;
  int current_index;
};

void initPoint2dVector(struct point2dVector *p2v) {
  p2v->current_max = 10;
  p2v->current_index = 0;
  p2v->vector = (struct PointRDP **)malloc(sizeof(struct PointRDP *) *
                                           (p2v->current_max));
}

void initPointVector(struct pointVector *pv) {
  pv->current_max = 10;
  pv->current_index = 0;
  pv->vector =
      (struct PointRDP *)malloc(sizeof(struct PointRDP) * (pv->current_max));
}

void resizePoint2dVector(struct point2dVector *p2v) {
  struct PointRDP **tmp;
  if ((tmp = (struct PointRDP **)realloc(
           p2v->vector, sizeof(struct PointRDP *) * (p2v->current_max * 2))) ==
      NULL) {
    free(p2v->vector);
    perror("malloc failed");
  }
  p2v->current_max *= 2;
  p2v->vector = tmp;
}

void resizePointVector(struct pointVector *pv) {
  struct PointRDP *tmp;
  if ((tmp = (struct PointRDP *)realloc(
           pv->vector, sizeof(struct PointRDP) * (pv->current_max * 2))) ==
      NULL) {
    free(pv->vector);
    perror("malloc failed");
  }
  pv->current_max *= 2;
  pv->vector = tmp;
}

void addPoint2dVector(struct point2dVector *p2v,
                      struct PointRDP *point_vector) {

  if (p2v->current_index + 1 >= p2v->current_max)
    resizePoint2dVector(p2v);
  p2v->vector[p2v->current_index] = point_vector;
  p2v->current_index += 1;
}

void addPointVector(struct pointVector *pv, struct PointRDP point) {

  if (pv->current_index + 1 >= pv->current_max)
    resizePointVector(pv);
  pv->vector[pv->current_index] = point;
  pv->current_index += 1;
}

struct PointRDP **trimPoint2dVector(struct point2dVector *p2v,
                                    int *vector_size) {
  struct PointRDP **tmp;
  if ((tmp = (struct PointRDP **)realloc(
           p2v->vector, sizeof(struct PointRDP *) * (p2v->current_index))) ==
      NULL) {
    free(p2v->vector);
    perror("malloc failed");
  }
  (*vector_size) = p2v->current_index;
  return tmp;
}

struct PointRDP *trimPointVector(struct pointVector *pv, int *vector_size) {
  struct PointRDP *tmp;
  if ((tmp = (struct PointRDP *)realloc(
           pv->vector, sizeof(struct PointRDP) * (pv->current_index))) ==
      NULL) {
    free(pv->vector);
    perror("malloc failed");
  }
  (*vector_size) = pv->current_index;
  return tmp;
}

// step around a pixel CCW
void stepCCW(struct PointRDP *current, struct PointRDP pivot) {
  if (current->x > pivot.x)
    setPoint(current, pivot.y - 1, pivot.x);
  else if (current->x < pivot.x)
    setPoint(current, pivot.y + 1, pivot.x);
  else if (current->y > pivot.y)
    setPoint(current, pivot.y, pivot.x + 1);
  else if (current->y < pivot.y)
    setPoint(current, pivot.y, pivot.x - 1);
}

// step around a pixel CW
void stepCW(struct PointRDP *current, struct PointRDP pivot) {
  if (current->x > pivot.x)
    setPoint(current, pivot.y + 1, pivot.x);
  else if (current->x < pivot.x)
    setPoint(current, pivot.y - 1, pivot.x);
  else if (current->y > pivot.y)
    setPoint(current, pivot.y, pivot.x - 1);
  else if (current->y < pivot.y)
    setPoint(current, pivot.y, pivot.x + 1);
}

// checks if a given pixel is out of bounds of the image
bool pixelOutOfBounds(struct PointRDP p, int numrows, int numcols) {
  return (p.x >= numcols || p.y >= numrows || p.x < 0 || p.y < 0);
}

// marks a pixel as examined after passing through
void markExamined(struct PointRDP mark, struct PointRDP center,
                  bool checked[4]) {
  // p3.y, p3.x + 1
  int loc = -1;
  //    3
  //  2 x 0
  //    1
  if (mark.x > center.x)
    loc = 0;
  else if (mark.x < center.x)
    loc = 2;
  else if (mark.y > center.y)
    loc = 1;
  else if (mark.y < center.y)
    loc = 3;

  if (loc == -1)
    perror("Error: markExamined Failed");

  checked[loc] = true;
  return;
}

// checks if given pixel has already been examined
bool isExamined(bool checked[4]) {
  // p3.y, p3.x + 1
  return checked[0];
}

void followBorder(int **image, int numrows, int numcols, int y, int x,
                  struct PointRDP p2, struct Border NBD,
                  struct point2dVector *contour_vector,
                  struct intVector *contour_counter) {
  struct PointRDP current;
  setPoint(&current, p2.y, p2.x);
  struct PointRDP start;
  setPoint(&start, y, x);

  //(3.1)
  // Starting from (i2, j2), look around clockwise the pixels in the
  // neighborhood of (i, j) and find a nonzero pixel.
  // Let (i1, j1) be the first found nonzero pixel. If no nonzero pixel is
  // found, assign -NBD to fij and go to (4).
  do {
    stepCW(&current, start);
    if (samePoint(current, p2)) {
      image[int(start.y)][int(start.x)] = -NBD.seq_num;
      struct PointRDP *temp =
          (struct PointRDP *)malloc(sizeof(struct PointRDP));
      temp[0] = start;
      addPoint2dVector(contour_vector, temp);
      addIntVector(contour_counter, 1);
      return;
    }
  } while (pixelOutOfBounds(current, numrows, numcols) ||
           image[int(current.y)][int(current.x)] == 0);

  struct pointVector point_storage;
  initPointVector(&point_storage);

  struct PointRDP p1 = current;

  //(3.2)
  //(i2, j2) <- (i1, j1) and (i3, j3) <- (i, j).

  struct PointRDP p3 = start;
  struct PointRDP p4;
  p2 = p1;
  bool checked[4];
  while (true) {
    //(3.3)
    // Starting from the next element of the pixel(i2, j2) in the
    // counterclockwise order, examine counterclockwise the pixels in the
    // neighborhood of the current pixel(i3, j3) to find a nonzero pixel and let
    // the first one be(i4, j4).
    current = p2;

    for (int i = 0; i < 4; i++)
      checked[i] = false;

    do {
      markExamined(current, p3, checked);
      stepCCW(&current, p3);
    } while (pixelOutOfBounds(current, numrows, numcols) ||
             image[int(current.y)][int(current.x)] == 0);
    p4 = current;

    // Change the value fi3, j3 of the pixel(i3, j3) as follows :
    //	If the pixel(i3, j3 + 1) is a 0 - pixel examined in the substep(3.3)
    //then fi3, j3 <- - NBD.
    //	If the pixel(i3, j3 + 1) is not a 0 - pixel examined in the substep(3.3)
    //and fi3, j3 = 1, then fi3, j3 ←NBD.
    //	Otherwise, do not change fi3, j3.

    if ((p3.x + 1 >= numcols || image[int(p3.y)][int(p3.x) + 1] == 0) &&
        isExamined(checked)) {
      image[int(p3.y)][int(p3.x)] = -NBD.seq_num;
    } else if (p3.x + 1 < numcols && image[int(p3.y)][int(p3.x)] == 1) {
      image[int(p3.y)][int(p3.x)] = NBD.seq_num;
    }

    addPointVector(&point_storage, p3);
    // printImage(image, image.size(), image[0].size());
    //(3.5)
    // If(i4, j4) = (i, j) and (i3, j3) = (i1, j1) (coming back to the starting
    // point), then go to(4);
    // otherwise, (i2, j2) <- (i3, j3), (i3, j3) <- (i4, j4), and go back
    // to(3.3).
    if (samePoint(start, p4) && samePoint(p1, p3)) {
      int vector_size;
      struct PointRDP *temp = trimPointVector(&point_storage, &vector_size);
      addPoint2dVector(contour_vector, temp);
      addIntVector(contour_counter, vector_size);
      return;
    }

    p2 = p3;
    p3 = p4;
  }
}

int **readFile(const char *s, int *numrows, int *numcols) {
  FILE *pFile;
  char header[100];

  pFile = fopen(s, "r");

  if (pFile == NULL) {
    perror("Error");
    exit(EXIT_FAILURE);
  }

  int c = 0;
  int r = 0;
  int temp = 0;
  int **image;
  if (pFile != NULL) {
    fscanf(pFile, "%s", header);
    fscanf(pFile, "%d %d", &c, &r);
    image = create2dArray(r, c);
    fscanf(pFile, "%s", header);
    for (int i = 0; i < r; i++) {
      for (int j = 0; j < c; j++) {
        fscanf(pFile, "%d", &temp);
        if (temp != 0)
          temp = 1;
        image[i][j] = temp;
      }
    }
  }

  (*numrows) = r;
  (*numcols) = c;

  return image;
}

// prints the hierarchy list
void printHierarchy(struct Node *hierarchy, int hierarchy_size) {
  for (int i = 0; i < hierarchy_size; i++) {
    printf("%2d:: parent: %3d first child: %3d next sibling: %3d\n", i + 1,
           hierarchy[i].parent, hierarchy[i].first_child,
           hierarchy[i].next_sibling);
    //		cout << setw(2) << i + 1 << ":: parent: " << setw(3) <<
    //hierarchy[i].parent << " first child: " << setw(3) <<
    //hierarchy[i].first_child << " next sibling: " << setw(3) <<
    //hierarchy[i].next_sibling << endl;
  }
}

void drawContour(struct PointRDP **contours, int *contour_index,
                 struct Pixel **color, int seq_num, struct Pixel pix) {
  int r, c;
  for (int i = 0; i < contour_index[seq_num]; i++) {
    r = contours[seq_num][i].y;
    c = contours[seq_num][i].x;
    color[r][c] = pix;
  }
}

struct Pixel chooseColor(int n) {

  struct Pixel tmp;
  switch (n % 6) {
  case 0:
    setPixel(&tmp, 255, 0, 0);
    return tmp;
  case 1:
    setPixel(&tmp, 255, 127, 0);
    return tmp;
  case 2:
    setPixel(&tmp, 255, 255, 0);
    return tmp;
  case 3:
    setPixel(&tmp, 0, 255, 0);
    return tmp;
  case 4:
    setPixel(&tmp, 0, 0, 255);
    return tmp;
  default:
    setPixel(&tmp, 139, 0, 255);
    return tmp;
  }
}

// creates a 2D array of struct Pixel, which is the 3 channel image needed to
// convert the 2D vector contours to a drawn bmp file
// uses DFS to step through the hierarchy tree, can be set to draw only the top
// 2 levels of contours, for example.
struct Pixel **createChannels(int h, int w, struct Node *hierarchy,
                              struct PointRDP **contours, int *contour_index,
                              int contour_size) {

  struct Pixel **color;

  if ((color = (struct Pixel **)malloc(sizeof(struct Pixel *) * h)) == NULL)
    perror("malloc failed");

  for (int i = 0; i < h; i++) {
    if ((color[i] = (struct Pixel *)malloc(sizeof(struct Pixel) * w)) == NULL)
      perror("malloc failed");
    memset(color[i], 0, sizeof(struct Pixel) * w);
  }

  for (int i = 1; i < contour_size; i++) {
    drawContour(contours, contour_index, color, i, chooseColor(i));
  }

  return color;
}

// save image to bmp
void saveImageFile(const char *file_name, int h, int w, struct Node *hierarchy,
                   struct PointRDP **contours, int *contour_index,
                   int contour_size) {
  FILE *f;

  struct Pixel **color =
      createChannels(h, w, hierarchy, contours, contour_index, contour_size);

  unsigned char *img = NULL;
  int filesize =
      54 + 3 * w * h; // w is your image width, h is image height, both int

  img = (unsigned char *)malloc(3 * w * h);
  memset(img, 0, 3 * w * h);
  int x, y;
  unsigned char r, g, b;
  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {
      y = (h - 1) - i;
      x = j;
      r = color[i][j].red;
      g = color[i][j].green;
      b = color[i][j].blue;
      /*	        if (r > 255) r=255;
      if (g > 255) g=255;
      if (b > 255) b=255;*/
      img[(x + y * w) * 3 + 2] = (unsigned char)(r);
      img[(x + y * w) * 3 + 1] = (unsigned char)(g);
      img[(x + y * w) * 3 + 0] = (unsigned char)(b);
    }
  }

  unsigned char bmpfileheader[14] = {'B', 'M', 0, 0,  0, 0, 0,
                                     0,   0,   0, 54, 0, 0, 0};
  unsigned char bmpinfoheader[40] = {40, 0, 0, 0, 0, 0, 0,  0,
                                     0,  0, 0, 0, 1, 0, 24, 0};
  unsigned char bmppad[3] = {0, 0, 0};

  bmpfileheader[2] = (unsigned char)(filesize);
  bmpfileheader[3] = (unsigned char)(filesize >> 8);
  bmpfileheader[4] = (unsigned char)(filesize >> 16);
  bmpfileheader[5] = (unsigned char)(filesize >> 24);

  bmpinfoheader[4] = (unsigned char)(w);
  bmpinfoheader[5] = (unsigned char)(w >> 8);
  bmpinfoheader[6] = (unsigned char)(w >> 16);
  bmpinfoheader[7] = (unsigned char)(w >> 24);
  bmpinfoheader[8] = (unsigned char)(h);
  bmpinfoheader[9] = (unsigned char)(h >> 8);
  bmpinfoheader[10] = (unsigned char)(h >> 16);
  bmpinfoheader[11] = (unsigned char)(h >> 24);

  f = fopen(file_name, "wb");
  fwrite(bmpfileheader, 1, 14, f);
  fwrite(bmpinfoheader, 1, 40, f);
  for (int i = 0; i < h; i++) {
    fwrite(img + (w * (i)*3), 3, w, f);
    fwrite(bmppad, 1, (4 - (w * 3) % 4) % 4, f);
  }

  free(img);
  for (int i = 0; i < h; i++) {
    free(color[i]);
  }
  free(color);
  fclose(f);
}
