
#ifndef _CLIP_H
#define _CLIP_H

typedef int8_t OutCode;

typedef struct cline {
    bool accept;
    int x1;
    int y1;
    int x2;
    int y2;
} clipline;

clipline clipLineToFrame(int x1, int y1, int x2, int y2,
			 int x_min, int x_max, int y_min, int y_max);

#endif
