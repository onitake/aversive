
    

typedef struct _Object_bb
{
	uint8_t x_min;
	uint8_t x_max;
	uint8_t y_min;
	uint8_t y_max;
	uint16_t len;
}Object_bb;

#define POLY_MAX_PTS 12
typedef struct _Object_poly
{	
	uint16_t pixels_perim;
	uint16_t pts_num;
	point_t pts[POLY_MAX_PTS];
	uint16_t len;
	uint8_t color;
}Object_poly;
    

typedef struct _zone
{	
	point_t p;
	uint8_t h:7;
	uint8_t valid:1;
}zone;


typedef struct _drop_column_zone
{
	uint8_t valid;
	uint8_t z;
	uint8_t h;
}drop_column_zone;

typedef struct _drop_lintel_zone
{
	uint8_t valid;
	uint8_t z1;
	uint8_t h;
	uint8_t z2;
}drop_lintel_zone;




#define MAX_ZONES 30

#define MAX_SISTER_PER_ZONE 1



unsigned char *parcour_img(unsigned char* data, int16_t x_in, int16_t y_in, Object_bb *sac_obj, Object_poly *sac_obj_poly, int16_t max_obj);

float vect_get_angle(vect_t*v, vect_t* w);

void object_poly_get_min_ar(Object_poly *o, unsigned int *pts_index_out, vect_t *v_out, vect_t *r1, vect_t*r2);

void draw_pt_vect(unsigned char *buf, int16_t x_in, int16_t y_in, vect_t *v, point_t p);

void vect_rot_trigo(vect_t* v);

int object_poly_caliper_to_rectangle(Object_poly *o, 
				     unsigned int *pts_index_out, vect_t* caliper, 
				     vect_t *r1, vect_t*r2, point_t *p);

int split_rectangle(point_t *p, vect_t *r1, vect_t* r2, uint8_t max_zone, zone* zones, uint8_t color);



int zone_filtre_min_surface(unsigned char* data, int16_t x_in, int16_t y_in, 
			    uint8_t color, unsigned int num_zone, zone* p);

void reset_drop_zone(void);
void display_drop_zones(uint8_t n_columns, uint8_t n_lintels, zone* zones);

unsigned int solve_objects_dropping(unsigned int points, unsigned int points_max,
				    uint8_t n_columns, uint8_t n_lintels, 
				    uint8_t zones_num, zone* zones, int8_t sisters[MAX_ZONES][MAX_SISTER_PER_ZONE], uint8_t working_zone);

uint8_t process_img(unsigned char *buffer, int16_t x_in, int16_t y_in,
		    zone * all_zones, uint8_t max_zones);


uint8_t color2h(uint8_t color);
uint8_t h2color(uint8_t color);

int8_t get_column_dropzone(unsigned char *buffer, int16_t x_in, int16_t y_in, 
			   uint8_t working_zone, int16_t center_x, int16_t center_y,
			   int16_t * dropzone_x, int16_t * dropzone_y);


uint8_t is_temple_there(unsigned char * buffer, int16_t x_in, int16_t y_in, 
			uint8_t h, int16_t center_x, int16_t center_y);



int8_t find_temple_dropzone(unsigned char *buffer, int16_t x_in, int16_t y_in, 
			    uint8_t working_zone, int16_t center_x, int16_t center_y,
			    int16_t * dropzone_x, int16_t * dropzone_y);

void process_img_to_zone(unsigned char *buffer, int16_t x_in, int16_t y_in);

#define PIXEL2CM (300./27.)


extern uint8_t g_zone_num;
extern zone g_all_zones[MAX_ZONES];
