typedef struct _line {
	double a;
	double b;
	double c;
} line_t;


void 
pts2line(const point_t *p1, const point_t *p2, line_t *l);

void
proj_pt_line(const point_t * p, const line_t * l, point_t * p_out);

uint8_t 
intersect_line(const line_t *l1, const line_t *l2, point_t *p);

uint8_t 
intersect_segment(const point_t *s1, const point_t *s2, 
		  const point_t *t1, const point_t *t2, 
		  point_t *p);
