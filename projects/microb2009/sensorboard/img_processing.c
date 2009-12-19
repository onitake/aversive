#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include <aversive/pgmspace.h>
#include <aversive/error.h>

#include <stdint.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <vect_base.h>
#include <lines.h>
#include <polygon.h>

#include "img_processing.h"


#define debug_printf(fmt, ...) printf_P(PSTR(fmt), ##__VA_ARGS__)

#ifndef HOST_VERSION

#include <aversive.h>
#include <pwm_ng.h>
#include <pid.h>
#include <time.h>
#include <quadramp.h>
#include <blocking_detection_manager.h>
#include <rdline.h>

#include <control_system_manager.h>
#include <adc.h>
#include <spi.h>
#include <ax12.h>

#include "main.h"

#define IMG_DEBUG(args...) DEBUG(E_USER_IMGPROCESS, args)
#define IMG_NOTICE(args...) NOTICE(E_USER_IMGPROCESS, args)
#define IMG_ERROR(args...) ERROR(E_USER_IMGPROCESS, args)

#else

#define IMG_DEBUG(args...) debug_printf(args)
#define IMG_NOTICE(args...) debug_printf(args)
#define IMG_ERROR(args...) debug_printf(args)

#endif




#define OBJECT_MINIMUM_DEMI_PERIMETER (2*3)
/* store object in pool if never seen 
 * returns:
 * 1 if new object
 * 0 if already known
 */
int store_obj(Object_bb* tab_o,  int total, Object_bb*o)
{
	uint8_t i;

	if (o->x_max - o->x_min + o->y_max - o->y_min < OBJECT_MINIMUM_DEMI_PERIMETER) 
		return 0;

	for (i=0;i<total;i++){
		if (!memcmp(&tab_o[i], o, sizeof(Object_bb)))
			return 0;
			
		if (tab_o[i].x_min==0 && tab_o[i].x_max==0 && 
		    tab_o[i].y_min==0 && tab_o[i].y_max==0){
			memcpy(&tab_o[i], o, sizeof(Object_bb));
			return 1;
		}
			
	}
	return 0;
}

/* step around object of given color and compute its bounding box */
void object_find_bounding_box(unsigned char* data, 
			      int16_t x_in, int16_t y_in, 
			      int16_t start_x, int16_t start_y, 
			      int16_t color, int16_t color_w,Object_bb* o)
{
	int16_t pos_x = start_x;
	int16_t pos_y = start_y;
	int16_t start =0;
	int16_t len = 0;
	int16_t count_stand = 0;

	vect_t v, vi;


	v.x = 1;
	v.y = 0;
    
	vi = v;
    
	o->x_max = 0;
	o->y_max = 0;
	o->x_min = x_in;
	o->y_min = y_in;
    
	
	while(1){
		if (pos_x == start_x && pos_y == start_y){
			count_stand++;
			if (count_stand>4)
				break;
			if( v.x == vi.x && v.y == vi.y && start)
				break;
		}
                
		if (pos_x<o->x_min)
			o->x_min = pos_x;
		if (pos_y<o->y_min)
			o->y_min = pos_y;
		if (pos_x>o->x_max)
			o->x_max = pos_x;
		if (pos_y>o->y_max)
			o->y_max = pos_y;
        
		/* is next pixel is good color */
		if (data[(pos_y+v.y)*x_in + pos_x+v.x] != color){
			pos_x = pos_x+v.x;
			pos_y = pos_y+v.y;
			len++;
			vect_rot_retro(&v);
			start = 1;
			continue;
		}    
		vect_rot_trigo(&v);
	}    
	
	o->len = len;
}





/* step around object of given color and computes its polygon */
void object_find_poly(unsigned char* data, 
		      int16_t x_in, int16_t y_in, 
		      int16_t start_x, int16_t start_y, 
		      int16_t color, int16_t color_w, Object_poly* o)
{
	int16_t pos_x = start_x;
	int16_t pos_y = start_y;
	int16_t start =0;
	int16_t len = 0;
	int16_t count_stand = 0;
	uint16_t pt_step, pt_num;
	vect_t v, vi;

	v.x = 1;
	v.y = 0;

	vi = v;    
	
	pt_step = o->len/POLY_MAX_PTS + 1;
	
	pt_num = 0;

	while(1){
		if (pos_x == start_x && pos_y == start_y){
			count_stand++;
			if (count_stand>4)
				break;
			if( v.x == vi.x && v.y == vi.y && start)
				break;
		}
        
		/* is next pixel is good color */
		if (data[(pos_y+v.y)*x_in + pos_x+v.x] != color){
			pos_x = pos_x+v.x;
			pos_y = pos_y+v.y;
			len++;
			
			if (len >pt_num*pt_step){
				o->pts[pt_num].x = pos_x;
				o->pts[pt_num].y = pos_y;
				pt_num+=1;
				if (pt_num>=POLY_MAX_PTS)
					break;
			}

			vect_rot_retro(&v);
			start = 1;
			continue;
		}    
		vect_rot_trigo(&v);
	}    

	o->pts_num = pt_num;

}

#define PT_LEFT 0
#define PT_RIGHT 2
#define PT_TOP 1
#define PT_DOWN 3

/* return most left/right/top/down pts indexes of given polygon */
void object_find_extrem_points_index(Object_poly* o, 
				     unsigned int *pts)
{
	unsigned int i;
	for (i=0;i<4;i++)
		pts[i] = 0;


	for (i=1;i<o->pts_num;i++){
		if (o->pts[i].x < o->pts[pts[PT_LEFT]].x)
			pts[PT_LEFT] = i;
		if (o->pts[i].x > o->pts[pts[PT_RIGHT]].x)
			pts[PT_RIGHT] = i;
		if (o->pts[i].y < o->pts[pts[PT_TOP]].y)
			pts[PT_TOP] = i;
		if (o->pts[i].y > o->pts[pts[PT_DOWN]].y)
			pts[PT_DOWN] = i;
	}

}


#define NUM_CALIPERS 4
/* for debug purpose: display a vector on image */
void draw_pt_vect(unsigned char *buf, int16_t x_in, int16_t y_in, 
		  vect_t *v, point_t p)
{
	unsigned int i;
	float n;
	int16_t x, y;
	float coef=1.0;

	if (!v->x && !v->y)
		return;

	n = vect_norm(v);
	coef = 1/n;
	for (i=0;i<5;i++){
		x = p.x;
		y = p.y;
		
		x+=(float)((v->x)*(float)i*(float)coef);
		y+=(float)((v->y)*(float)i*(float)coef);
		if ((x== p.x) && (y == p.y))
			buf[y*x_in+x] = 0x0;
		else
			buf[y*x_in+x] = 0x0;
	}
	
}

#define CAL_X 3
#define CAL_Y 0


/* compute minimum rectangle area including the given convex polygon */
void object_poly_get_min_ar(Object_poly *o, unsigned int *pts_index_out, 
                            vect_t *v_out, vect_t *r1, vect_t*r2)
{
	vect_t calipers[NUM_CALIPERS];
	vect_t edges[NUM_CALIPERS];

	unsigned int i;
	unsigned int calipers_pts_index[NUM_CALIPERS];
	float angles[NUM_CALIPERS];
	float min_angle;
	float total_rot_angle = 0;
	int caliper_result_index;

	vect_t res1, res2;
	float ps, n1, n2, caliper_n;

	float aera_tmp;
	/* XXX hack sould be max*/
	float aera_min=0x100000;

	object_find_extrem_points_index(o, calipers_pts_index);
		
	calipers[0].x = 0;
	calipers[0].y = 1;

	calipers[1].x = -1;
	calipers[1].y = 0;

	calipers[2].x = 0;
	calipers[2].y = -1;

	calipers[3].x = 1;
	calipers[3].y = 0;


	while (total_rot_angle <= M_PI/2){

		for (i=0;i<NUM_CALIPERS;i++){
			/* compute polygon edge vector */
			edges[i].x = o->pts[(calipers_pts_index[i] + 1)%o->pts_num].x - 
				o->pts[calipers_pts_index[i]].x;
			edges[i].y = o->pts[(calipers_pts_index[i] + 1)%o->pts_num].y - 
				o->pts[calipers_pts_index[i]].y;

			/* compute angle between caliper and polygon edge */
			angles[i] = vect_get_angle(&edges[i], &calipers[i]);
		}

		/* find min angle */
		min_angle = angles[0];
		caliper_result_index = 0;
		for (i=1;i<NUM_CALIPERS;i++){
			if (angles[i]<min_angle){
				min_angle = angles[i];
				caliper_result_index = i;
			}
		}

		/* rotate calipers */
		calipers[caliper_result_index] = edges[caliper_result_index];
		
		for (i=caliper_result_index; i<caliper_result_index + o->pts_num; i++){
			calipers[(i+1) % NUM_CALIPERS] = calipers[i % NUM_CALIPERS];
			vect_rot_trigo(&calipers[(i+1) % NUM_CALIPERS]);
		}

		/* update calipers point */
		for (i=0;i<NUM_CALIPERS;i++){
			if (angles[i]==min_angle)
				calipers_pts_index[i] = (calipers_pts_index[i] + 1) % o->pts_num;
		}
		
		res1.x = o->pts[calipers_pts_index[2]].x - o->pts[calipers_pts_index[0]].x;
		res1.y = o->pts[calipers_pts_index[2]].y - o->pts[calipers_pts_index[0]].y;

		res2.x = o->pts[calipers_pts_index[3]].x - o->pts[calipers_pts_index[1]].x;
		res2.y = o->pts[calipers_pts_index[3]].y - o->pts[calipers_pts_index[1]].y;
		
		ps = vect_pscal(&res1, &calipers[CAL_X]);
		n1 = vect_norm(&res1);
		caliper_n = vect_norm(&calipers[CAL_X]);
		caliper_n*=caliper_n;

		res1 = calipers[CAL_X];
		
		res1.x *= ps/(caliper_n);
		res1.y *= ps/(caliper_n);


		ps = vect_pscal(&res2, &calipers[CAL_Y]);
		n1 = vect_norm(&res2);

		res2 = calipers[CAL_Y];
		
		res2.x *= ps/(caliper_n);
		res2.y *= ps/(caliper_n);
		
		n1 = vect_norm(&res1);
		n2 = vect_norm(&res2);
		
		aera_tmp = n1*n2;
		
		if (aera_min >aera_tmp){
			aera_min = aera_tmp;
			for (i=0;i<NUM_CALIPERS;i++){
				pts_index_out[i] = calipers_pts_index[i];
			}
			*v_out = calipers[0];
			*r1 = res1;
			*r2 = res2;
			
		}
		total_rot_angle+=min_angle;
	}

	return;
}

#define COEF_CALIP 1
/* transform caliper to rectangle coordinates */
int object_poly_caliper_to_rectangle(Object_poly *o, 
				     unsigned int *pts_index_out, vect_t* caliper, 
				     vect_t *r1, vect_t*r2, point_t *p)
{
	line_t l1, l2;
	vect_t caliper_tmp;
	int ret, i;
	double mp_x, mp_y;
	point_t p_int1;//, p_int2;

	point_t p1, p2;

	caliper_tmp = *caliper;

	//IMG_DEBUG("cal: %" PRIi32 " %" PRIi32 "", caliper_tmp.x, caliper_tmp.y);
  
	mp_x = 0;
	mp_y = 0;

	/* to be precise, calc 4 intersection of 4 calipers */
	for (i=0;i<NUM_CALIPERS;i++){
		p1.x = o->pts[pts_index_out[i]].x;
		p1.y = o->pts[pts_index_out[i]].y;
  
		p2.x = p1.x+COEF_CALIP*caliper_tmp.x;
		p2.y = p1.y+COEF_CALIP*caliper_tmp.y;
  
		pts2line(&p1, &p2, &l1);
  	
		vect_rot_trigo(&caliper_tmp);  
  
		p1.x = o->pts[pts_index_out[(i+1)%NUM_CALIPERS]].x;
		p1.y = o->pts[pts_index_out[(i+1)%NUM_CALIPERS]].y;
  
		p2.x = p1.x+COEF_CALIP*caliper_tmp.x;
		p2.y = p1.y+COEF_CALIP*caliper_tmp.y;
  
		pts2line(&p1, &p2, &l2);
  	
		ret = intersect_line(&l1, &l2, &p_int1);
		if (ret!=1)
			return 0;
		//IMG_DEBUG("int1 (%d): %" PRIi32 " %" PRIi32 " ", ret, p_int1.x, p_int1.y);
  
		mp_x+=p_int1.x;
		mp_y+=p_int1.y;
        
	}
  
  
  
	p->x = lround(mp_x/NUM_CALIPERS);
	p->y = lround(mp_y/NUM_CALIPERS);
  

	return 1;

}

#define OBJECT_DIM 5
/* split zone in many column's sized area */
int split_rectangle(point_t *p, vect_t *r1, vect_t* r2, uint8_t max_zone, zone* zones, uint8_t color)
{
	int n1, n2;
	int i, j;
	int index=0;
	int r1_s, r2_s;
	point_t ptmp;
  
  
	n1 = vect_norm(r1);
	n2 = vect_norm(r2);

	r1_s = n1/OBJECT_DIM;
	r2_s = n2/OBJECT_DIM;

	if (!r1_s || ! r2_s)
		return 0;
  
	ptmp.x = p->x - r1->x/2 - r2->x/2 + (r1->x/(r1_s*2))+(r2->x/(r2_s*2));
	ptmp.y = p->y - r1->y/2 - r2->y/2 + (r1->y/(r1_s*2))+(r2->y/(r2_s*2));
  
	for(i=0;i<r1_s;i++){
		for(j=0;j<r2_s;j++){
			zones[index].p.x = ptmp.x + (i*r1->x)/r1_s+(j*r2->x)/r2_s;
			zones[index].p.y = ptmp.y + (i*r1->y)/r1_s+(j*r2->y)/r2_s;
			zones[index].h = color;
			zones[index].valid = 1;

			index++;
			if (index>=max_zone)
				return index;


		}
	}
    
	return index;
}

#define OBJECT_SEMI_DIM (OBJECT_DIM/2)
#define MIN_SURFACE_PERCENT 50
#define HIGHER_MAX_PIXEL 5


int zone_has_enought_pixels(unsigned char* data, int16_t x_in, int16_t y_in, zone* z)
{
	int x, y;
	uint16_t count, total_pix, higher_pixels;

	count = 0;
	total_pix=0;
	higher_pixels = 0;

	for (x = -OBJECT_SEMI_DIM; 
	     (x <= OBJECT_SEMI_DIM) && (higher_pixels < HIGHER_MAX_PIXEL); 
	     x++){
		for (y = -OBJECT_SEMI_DIM; 
		     (y <= OBJECT_SEMI_DIM) && (higher_pixels < HIGHER_MAX_PIXEL); 
		     y++){
			total_pix++;
			if (data[x_in * (y + z->p.y) + x + z->p.x] == z->h)
				count++;
			
			if (data[x_in * (y + z->p.y) + x + z->p.x] > z->h)
				higher_pixels++;
			
		}
		
	}

	IMG_DEBUG("has enougth pixel (h: %d x %"PRIi32": y:%"PRIi32") total: %d/%d (tt: %d, hmax: %d)", z->h, z->p.x, z->p.y, 
		  count, (total_pix *  MIN_SURFACE_PERCENT) / 100, total_pix, higher_pixels);
			
	if ((count > (total_pix *  MIN_SURFACE_PERCENT) / 100) && 
	    (higher_pixels <HIGHER_MAX_PIXEL))
		return 1;
	
	return 0;
}

int zone_filter_min_surface(unsigned char* data, int16_t x_in, int16_t y_in, 
			    uint8_t color, unsigned int zones_num, zone* p)
{
	int i;
	
	for (i = 0; i < zones_num ; i++){
		if (zone_has_enought_pixels(data, x_in, y_in, &p[i]))
			continue;

		p[i].valid = 0;		
	}

	IMG_NOTICE("num zone after min surf: %d", zones_num);
	
	return zones_num;
	
}

/* center is 15 cm radius*/
#define CENTER_RADIUS 15

/* the complete column must be in the drop zone*/
#define CENTER_MAX_DIST (15-3)

/* the column must not be too close from center*/
#define CENTER_MIN_DIST (8)

int zone_filter_center(unsigned int zones_num, zone* p, int16_t center_x, int16_t center_y, int tweak_min_margin)
{
	int i;
	vect_t v;
	
	for (i = 0; i < zones_num; i++){
		
		v.x = p[i].p.x - center_x;
		v.y = p[i].p.y - center_y;
		IMG_DEBUG("square dist to center %"PRIi32" (%d %d)",
			  v.x*v.x + v.y*v.y,  (CENTER_MIN_DIST+tweak_min_margin) * (CENTER_MIN_DIST+tweak_min_margin), CENTER_MAX_DIST * CENTER_MAX_DIST);

		if (v.x*v.x + v.y*v.y < CENTER_MAX_DIST * CENTER_MAX_DIST && 
		    v.x*v.x + v.y*v.y > (CENTER_MIN_DIST+tweak_min_margin) * (CENTER_MIN_DIST+tweak_min_margin))
			continue;

		p[i].valid = 0;

	}

	return zones_num;
}

#define MAX_DIST_TO_ZONE 2

unsigned int zone_filter_zone_rect(unsigned int zones_num, zone* p, int16_t center_x, int16_t center_y , uint8_t working_zone)
{
	int i;
	
	for (i = 0; i < zones_num; i++){		
		
		IMG_DEBUG("rct x:%"PRIi32" y:%"PRIi32" (centerx: %d)",p[i].p.x , p[i].p.y,  center_x);
		
		if ((p[i].p.x > center_x - MAX_DIST_TO_ZONE) && (p[i].h > working_zone))
			continue;

		p[i].valid = 0;
	}

	return zones_num;
}


/* delete point to render polygon convex */
int object_poly_to_convex(Object_poly *o)
{
	unsigned int i, j;
	vect_t v, w;
	int16_t z;
	unsigned int del_pts_num = 0;
	
	for (i=0;i<o->pts_num;){
		v.x = o->pts[(i + o->pts_num - 1)%o->pts_num].x - o->pts[i].x;
		v.y = o->pts[(i + o->pts_num - 1)%o->pts_num].y - o->pts[i].y;

		w.x = o->pts[(i+1)%o->pts_num].x - o->pts[i].x;
		w.y = o->pts[(i+1)%o->pts_num].y - o->pts[i].y;
		
		z = vect_pvect(&v, &w);
		if (z>0){
			i+=1;
			continue;
		}
		
		/* found a convex angle (or colinear points) */
		for (j = i; j < o->pts_num-1; j++){
			o->pts[j] = o->pts[j+1];
		}
		if (i!=0)
			i-=1;
		o->pts_num--;
		del_pts_num++;
	}

	return del_pts_num;
}





#define DEFAULT_COLOR 255
/* scan all image and find objects*/
unsigned char *parcour_img(unsigned char* data, int16_t x_in, int16_t y_in, 
			   Object_bb *sac_obj, Object_poly *sac_obj_poly, int16_t max_obj)
{
	int16_t i, obj_num;

	uint8_t in_color=0;    
	int16_t etat;

	Object_bb o;
	int ret;

	obj_num = 0;
	/*
	  first, delete borders
	*/
	for (i=0;i<x_in;i++){
		data[i] = 0;
		data[(y_in - 1) * x_in + i] = 0;
	}

	for (i=0;i<y_in;i++){
		data[i * x_in] = 0;
		data[i * x_in + x_in - 1] = 0;
	}
      

    
	etat = 0; 
	/*
	  0 look for color (object or edge)
	  1 look for edge end
	*/
    
	for (i=1;i<x_in*y_in;i++){
		switch(etat){
		case 0:
			//we are in the dark
			switch(data[i]){
			case 0:
				//look for in dark
				break;
				/*
				  case 1:
				  case 2:
				  case 0x15:
				  case 0x24:
				*/
			default:
				in_color = data[i];
				etat = 1;
				// we found an object
				object_find_bounding_box(data, x_in, y_in, (i-1)%x_in, (i-1)/x_in, data[i], 255, &o);

				ret = store_obj(sac_obj,  max_obj, &o);
				/* if new object, process rotating calipers */
				if (ret){
					sac_obj_poly[obj_num].len = o.len;
					object_find_poly(data, x_in, y_in, 
							 (i-1)%x_in, (i-1)/x_in, 
							 data[i], 255, &sac_obj_poly[obj_num]);
					IMG_DEBUG("%d",sac_obj_poly[obj_num].pts_num);
					object_poly_to_convex(&sac_obj_poly[obj_num]);
					/*
					  for (j=0;j<sac_obj_poly[obj_num].pts_num;j++){
					  data[sac_obj_poly[obj_num].pts[j].y*x_in + sac_obj_poly[obj_num].pts[j].x] = 8;
					  }*/
					sac_obj_poly[obj_num].color = data[i];
					obj_num++;
				}
                        
				break;

			case DEFAULT_COLOR:
				//we must out of color
				break;
			}    
			break;

		case 1:
			if (data[i] != in_color){
				i--;
				etat = 0;
			}
			/*
			  we are in a color and want to go out of it
			*/
			break;

		}    
        
        
        
	}    

    
	return data;
    
    
}    
/* space between twin tower is 13 pixels*/
#define SPACE_INTER_TWIN_TOWER (13)

#define SPACE_INTER_TWIN_TOWER_TOLERENCE 3



/* find best twin tower for each zone */
void find_twin_tower(uint8_t zones_num, zone* zones, int8_t sisters[MAX_ZONES][MAX_SISTER_PER_ZONE], 
		     int16_t center_x, int16_t center_y)
{

	uint8_t i, j;
	uint8_t z_n;
	int n1, n2;
	unsigned int z1, z2, z3;
	//int32_t scal1, scal2;
	vect_t v, v1, v2;
	line_t l;
	point_t p;
	unsigned int good_zone;
	double dist, dist2;
	unsigned int current_sister;

	/* init dummy sisters */
	for (i = 0; i < zones_num; i++)
		for (j = 0; j < MAX_SISTER_PER_ZONE; j++)
			sisters[i][j] = -1;



	for (z_n = 0; z_n < zones_num; z_n++){
		if (!zones[z_n].valid)
			continue;

		current_sister = 0;

            	for (i = 0; i < zones_num; i++){


			/* we already have max sisters */
			if (current_sister >= MAX_SISTER_PER_ZONE)
				break;


			if (!zones[i].valid)
				continue;


            		if (i == z_n)
            			continue;
            
            		/* twin tower must have same high */
            		if (zones[i].h != zones[z_n].h)
            			continue;

			IMG_DEBUG("test sisters (%"PRIi32" %"PRIi32") (%"PRIi32" %"PRIi32")", 
				  zones[z_n].p.x,   zones[z_n].p.y,
				  zones[i].p.x,   zones[i].p.y);
			
			
            		dist = sqrt( (zones[i].p.x - zones[z_n].p.x) * (zones[i].p.x - zones[z_n].p.x) + 
				     (zones[i].p.y - zones[z_n].p.y) * (zones[i].p.y - zones[z_n].p.y) );
			
            		
			IMG_DEBUG("sister space is %2.2f may be near %d", dist, SPACE_INTER_TWIN_TOWER);
            		
            		/* 
			   twin tower must be close/far enought to drop lintel
			 */
            		if (ABS(dist - SPACE_INTER_TWIN_TOWER) > SPACE_INTER_TWIN_TOWER_TOLERENCE)
            			continue;


            		pts2line(&zones[i].p, &zones[z_n].p, &l);
			
			/* 
			   test the paralelism of the temple:
			   zone may be on same distance from center 
			*/
			

			v1.x = zones[z_n].p.x - center_x;
			v1.y = zones[z_n].p.y - center_y;
			
			dist = vect_norm(&v1);

			v2.x = zones[i].p.x - center_x;
			v2.y = zones[i].p.y - center_y;
			
			dist2 = vect_norm(&v2);

			IMG_DEBUG("zone dist %2.2f %2.2f", dist, dist2);
			if (ABS(dist-dist2) > 3){
            			IMG_DEBUG("bad parallelism %2.2f", ABS(dist-dist2));
				continue;
			}
				
			

            
            		/* no other aligned tower to avoid dropping on a lintel 
            		 *  (3 aligned zone may mean lintel) 
            		 */

            		good_zone = 1;
            
            		for (j = 0; j < zones_num; j++){
            			if (j==i ||j == z_n)
            				continue;
            
            			/* if third zone, but lower */
            			if (zones[j].h <= zones[i].h)
            				continue;
            
            			/* 
            			   check distance from dropping zone to
            			   line formed by twin towers
            			*/
            
            			proj_pt_line(&zones[j].p, &l, &p);


				/* test if projected point is in the segement */
				

            			v.x = zones[z_n].p.x - zones[i].p.x;
            			v.y = zones[z_n].p.y - zones[i].p.y;

            			v1.x = p.x - zones[i].p.x;
            			v1.y = p.y - zones[i].p.y;

				n1 = vect_pscal_sign(&v, &v1);

            			v.x = -v.x;
            			v.y = -v.y;

            			v1.x = p.x - zones[z_n].p.x;
            			v1.y = p.y - zones[z_n].p.y;
				

				n2 =vect_pscal_sign(&v, &v1);

            			v.x = p.x - zones[j].p.x;
            			v.y = p.y - zones[j].p.y;
            
            			dist = vect_norm(&v);
            			IMG_DEBUG("dist pt  h %d n: (%d %d) (%"PRIi32" %"PRIi32") to line %2.2f", zones[j].h, n1, n2, zones[j].p.x, zones[j].p.y, dist);

            
            			if ((n1>=0 && n2>=0) && dist < OBJECT_DIM+2.){
            				good_zone = 0;
            				break;
            			}
            				
            
            			/* test if zone is far from points*/
            			
            			v1.x = zones[j].p.x - zones[z_n].p.x;
            			v1.y = zones[j].p.y - zones[z_n].p.y;
            
            			dist = vect_norm(&v1);
            			IMG_DEBUG("dist pt to z1 %2.2f", dist);
            
            			if (dist < OBJECT_DIM){
            				good_zone = 0;
            				break;
            			}
            			
            			v2.x = zones[j].p.x - zones[i].p.x;
            			v2.y = zones[j].p.y - zones[i].p.y;
            
            
            			dist = vect_norm(&v2);
            			IMG_DEBUG("dist pt to z2 %2.2f", dist);
            
            			if (dist < OBJECT_DIM){
            				good_zone = 0;
            				break;
            			}

				
            
            			z1 = i;
            			z2 = z_n;
            			z3 = j;
            

				
            
            
            			/*
            			  XXX may be a lintel on lintel !!
            			 */

            		}
            
            		if (!good_zone)
            			continue;
            
			IMG_DEBUG("sisters ok (%"PRIi32" %"PRIi32") (%"PRIi32" %"PRIi32")", 
				  zones[z_n].p.x,   zones[z_n].p.y,
				  zones[i].p.x,   zones[i].p.y);

			
            		sisters[z_n][current_sister] = i;
            		current_sister++;
            	}
	}
}

/* test if a higher zone is too close */
int test_close_zone(uint8_t zones_num, zone* zones, unsigned int z_n)
{
	uint8_t i;
	vect_t v;
	double dist;

	for (i = 0; i < zones_num; i++){
		if (i == z_n)
			continue;
		if (zones[i].h <= zones[z_n].h)
			continue;
		
		v.x = zones[i].p.x - zones[z_n].p.x;
		v.y = zones[i].p.y - zones[z_n].p.y;

		dist = vect_norm(&v);
		//IMG_DEBUG("dist pt to pt %2.2f", dist);
			
		if (dist < OBJECT_DIM){
			return 1;
		}

	}

	return 0;
}

#define MAX_COLUMN 4
#define MAX_LINTEL 2

drop_column_zone drop_c[MAX_COLUMN];
drop_lintel_zone drop_l[MAX_LINTEL];


void reset_drop_zone(void)
{
	memset(drop_c, 0, sizeof(drop_c));
	memset(drop_l, 0, sizeof(drop_l));

}


void display_drop_zones(uint8_t n_columns, uint8_t n_lintels, zone* zones)
{
	unsigned int i;

	for (i=0;i<n_columns;i++)
		IMG_NOTICE("c %d:(h:%d) (%"PRIi32" %"PRIi32") valid=%d",
			     i, drop_c[i].h,
			     zones[drop_c[i].z].p.x, zones[drop_c[i].z].p.y,
			     drop_c[i].valid);

	for (i=0;i<n_lintels;i++)
		IMG_NOTICE("l %d:(h:%d) (%"PRIi32" %"PRIi32") "
			     "(%"PRIi32" %"PRIi32") valid=%d",
			     i, drop_l[i].h,
			     zones[drop_l[i].z1].p.x, zones[drop_l[i].z1].p.y, 
			     zones[drop_l[i].z2].p.x, zones[drop_l[i].z2].p.y, drop_l[i].valid);
	
}

#define MY_MAX(a, b) ((a)>(b)?(a):(b))


#if 0
#define MAX_DROP_HIGH  8


/* 
   recursive function to maximize points during object
   dropping, given lintel/column number
   working zone may be 1, 2 or 3
 */
unsigned int solve_objects_dropping(unsigned int points, unsigned int points_max,
				    uint8_t n_columns, uint8_t n_lintels, 
				    uint8_t zones_num, zone* zones, int8_t sisters[MAX_ZONES][MAX_SISTER_PER_ZONE], uint8_t working_zone)
{
	
	uint8_t i, j;
	unsigned int points_calc;
	//unsigned int points_added = 0;
	int sister;
	int ret;


	/* if no more objects, return points */
	if (n_columns == 0 && n_lintels == 0)
		return MY_MAX(points, points_max);
	
	/* start by putting columns if so */
	for (i = 0; i < zones_num; i++){
		if (zones[i].h >= MAX_DROP_HIGH)
			continue;

		if (n_columns){

			ret = test_close_zone(zones_num, zones, i);
			if (ret)
				continue;

			zones[i].h++;
			points_calc = solve_objects_dropping(points + zones[i].h, points_max,
							     n_columns - 1, n_lintels, 
							     zones_num, zones, sisters, working_zone);
			
			if (points_calc > points_max){
				points_max = points_calc;
				drop_c[n_columns - 1].z = i;
				drop_c[n_columns - 1].h = zones[i].h;
				drop_c[n_columns - 1].valid  = 1;
				
			}
			zones[i].h--;
		}
		/* we must depose all columns before dropping lintels */
		else if (n_lintels){
			
			/* dont drop lintel on ground */
			if (zones[i].h  <= working_zone)
				continue;

			/* XXX todo need get second zone near selected one */
			//ret = find_twin_tower(zones_num, zones, i, &sister);

			for (j = 0; j < MAX_SISTER_PER_ZONE; j++){
				sister = sisters[i][j];
				if (sister == -1)
					break;
				if (zones[i].h != zones[sister].h){
					sister = -1;
				}
				
			}

			if (sister == -1)
				continue;
			//IMG_DEBUG("sister found: %d %d (h=%d %p)", i, sister, zones[i].h, &zones[i].h);
			
			zones[i].h++;
			zones[sister].h++;

			
			points_calc = solve_objects_dropping(points + zones[i].h * 3, points_max,
							     n_columns, n_lintels - 1, 
							     zones_num, zones, sisters, working_zone);
			
			if (points_calc > points_max){
				points_max = points_calc;

				drop_l[n_lintels - 1].z1 = i;
				drop_l[n_lintels - 1].z2 = sister;
				drop_l[n_lintels - 1].h = zones[i].h;
				drop_l[n_lintels - 1].valid = 1;
				
			}

			
			zones[sister].h--;
			zones[i].h--;
		}
	}
	
	return MY_MAX(points, points_max);
}
#endif


/*  */
int find_column_dropzone(uint8_t zones_num, zone* zones)
{
	uint8_t i;
	
	uint8_t z_n = 0;

	if (zones_num <= 0)
		return -1;

	for (i = 0; i < zones_num; i++){
		if (!zones[i].valid)
			continue;
		if (zones[i].h > zones[z_n].h)
			z_n = i;
	}


	/* 
	   now, chose dropzone closest to robot 
	   meaning little x, big y
	   so maximise y-x
	 */
	for (i = 0; i < zones_num; i++){
		if (zones[i].h != zones[z_n].h)
			continue;
		if (!zones[i].valid)
			continue;
		if (zones[i].p.y - zones[i].p.x > zones[z_n].p.y - zones[z_n].p.x)
			z_n = i;
	}
	
	
	
	return z_n;
}



uint8_t color2h(uint8_t color)
{
	return (0x100-color)/0x20;
}

uint8_t h2color(uint8_t color)
{
	return color*0x20;
}


#define NUM_ZONE_GENERATE 8
#define DIST_ZONE_GEN 9

/*
  remove zone at ground level, and generate zones on 
  a circle at X cm from center
 */
unsigned int generate_center_ground_zones(unsigned char* data, int16_t x_in, int16_t y_in,
					  zone * zones, unsigned int zones_num, uint8_t max_zones, int16_t center_x, int16_t center_y)
{
	double c_a, s_a;
	uint8_t i, j;
	double px1, py1, px2, py2;

	
	/* first del zone at level 2 */
	for (i = 0; i < zones_num; ){
		if (zones[i].h!=2){
			i++;
			continue;
		}

		for (j = i; j < zones_num-1; j++)
			zones[j] = zones[j+1];

		zones_num--;

	}

	/* generate zones around circle  */

	c_a = cos(2*M_PI/NUM_ZONE_GENERATE);
	s_a = sin(2*M_PI/NUM_ZONE_GENERATE);

	px1 = DIST_ZONE_GEN;
	py1 = 0;

	for (i = 0; i < NUM_ZONE_GENERATE; i++){
		
		zones[zones_num].p.x = center_x + px1;
		zones[zones_num].p.y = center_y + py1;
		zones[zones_num].h = 2;
		zones[zones_num].valid = 1;

		
		px2 = px1*c_a + py1*s_a;
		py2 = -px1*s_a + py1*c_a;

		px1 = px2;
		py1 = py2;

		/* skip zone if it is not in img */
		if (zones[zones_num].p.x < 0 ||  zones[zones_num].p.y < 0 ||
		    zones[zones_num].p.x >= x_in ||  zones[zones_num].p.y > y_in)
			continue;

		/* skip zone if not enougth pixels */
		if (!zone_has_enought_pixels(data, x_in, y_in, &zones[zones_num]))
			continue;
		
		zones_num++;
		if (zones_num >= max_zones)
			break;
		
	}

	return zones_num;

	
}


/*
  remove zone at ground level, and generate zones on 
  a line at X cm from robot
 */
unsigned int generate_rectangle_ground_zones(unsigned char* data, int16_t x_in, int16_t y_in,
					     zone * zones, unsigned int zones_num, uint8_t max_zones, int16_t center_x, int16_t center_y,
					     uint8_t working_zone)
{
	uint8_t i, j;
	uint8_t y;

	/* first del zone at level i */
	for (i = 0; i < zones_num; ){
		if (zones[i].h != working_zone ){
			i++;
			continue;
		}

		for (j = i; j < zones_num-1; j++)
			zones[j] = zones[j+1];

		zones_num--;
	}


	/* generate zones on a line  */
	for (y = OBJECT_DIM; y < y_in; y+=OBJECT_DIM){

		zones[zones_num].p.x = center_x;
		zones[zones_num].p.y = y;
		zones[zones_num].h = working_zone ;
		zones[zones_num].valid = 1;

		if (!zone_has_enought_pixels(data, x_in, y_in, &zones[zones_num]))
			continue;
		zones_num++;

		if (zones_num >= max_zones)
			break;
		
	} 
	return zones_num;
	
}


#define MAX_DECAL_LINE 5
#define ENOUGHT_ZONE_PIXEL 2

void recal_img_y(unsigned char* buffer, int16_t x_in, int16_t y_in, 
		 uint8_t working_zone)
{
	uint8_t i, j;
	uint8_t cpt;

	/* recal img only for central zone */
	if (working_zone !=2)
		return;
	
	for (i = 0; i < MAX_DECAL_LINE; i++){
		cpt = 0;
		for (j = 0; j < x_in; j++){
			if (buffer[i*x_in + j] ==2)
				cpt++;
		}

		if (cpt >= ENOUGHT_ZONE_PIXEL)
			break;
	}

	memmove(buffer, &buffer[i * x_in], x_in * y_in - i*x_in);
	memset(&buffer[x_in * y_in - i * x_in], 0,  i*x_in);
}


#define MAX_OBJECTS 20

#define MAX_ZONES_PER_OBJECT 20


uint8_t g_zone_num;
zone g_all_zones[MAX_ZONES];

uint8_t process_img(unsigned char *buffer, int16_t x_in, int16_t y_in,
		    zone * all_zones, uint8_t max_zones)
{

	int ret;
	int i, j;

	zone zones[MAX_ZONES_PER_OBJECT];
  
	vect_t caliper;
	unsigned int pts_cal[4];
	point_t ptmp;
	vect_t r1, r2;
	int zone_len;
  
  
	uint8_t zone_num = 0;

	Object_bb sac_obj[MAX_OBJECTS];
	Object_poly sac_obj_poly[MAX_OBJECTS];
  


	/* 
	   XXX fix: only decal for working zone 2/(1?) 
	   but we dont have info yet
	*/
	recal_img_y(buffer, x_in, y_in, 2);

	memset(sac_obj, 0, sizeof(sac_obj));
	memset(sac_obj_poly, 0, sizeof(sac_obj_poly));


	/* first, find polygons*/
	parcour_img(buffer, x_in, y_in, sac_obj, sac_obj_poly, MAX_OBJECTS);

	/* enclose each poygon in the min area polygon
	   then, split each rectangle in dropping zone
	*/
	for (i=0;i<MAX_OBJECTS;i++){

		if (!sac_obj_poly[i].pts_num)
			continue;
		
		IMG_DEBUG("obj: %d %d %d %d %d", 
			  i, 
			  sac_obj[i].x_min, 
			  sac_obj[i].y_min, 
			  sac_obj[i].x_max, 
			  sac_obj[i].y_max);
		
		//IMG_DEBUG("poly pts_num: %d", sac_obj_poly[i].pts_num);
        
		object_poly_get_min_ar(&sac_obj_poly[i], &pts_cal[0], &caliper, &r1, &r2);
    
		ret = object_poly_caliper_to_rectangle(&sac_obj_poly[i], &pts_cal[0], &caliper,
						       &r1, &r2, &ptmp);
    
		if (!ret)
			continue;

		/*
		IMG_DEBUG("r: (%3"PRIi32" %3"PRIi32")  "
			     "(%3"PRIi32" %3"PRIi32")",
			     r1.x, r1.y, r2.x, r2.y);
		IMG_DEBUG("intersection: %"PRIi32" %"PRIi32"",
			     ptmp.x, ptmp.y);
		*/
    
		zone_len = split_rectangle(&ptmp, &r1, &r2, 
					      MAX_ZONES_PER_OBJECT, &zones[0], sac_obj_poly[i].color);
		//IMG_DEBUG("split ok %d", zone_len);
    
		zone_len =  zone_filter_min_surface(buffer, x_in, y_in, 
						    sac_obj_poly[i].color, 
						    zone_len, &zones[0]);		
		
		for (j = 0; j < zone_len && zone_num < max_zones; zone_num++, j++)
			all_zones[zone_num] = zones[j];

	}


	IMG_NOTICE("num zones end: %d", zone_num);
	
	return zone_num;
}


void process_img_to_zone(unsigned char *buffer, int16_t x_in, int16_t y_in)
{
	g_zone_num = process_img(buffer, x_in, y_in,
				 g_all_zones, MAX_ZONES);
}

uint8_t filter_zones(unsigned char *buffer, int16_t x_in, int16_t y_in,
		     zone * all_zones, uint8_t zone_num, uint8_t max_zones,
		     uint8_t working_zone, int16_t center_x, int16_t center_y,
		     int tweak_min_margin)
{
	uint8_t i;

	/* first valid all zones */
	for (i = 0; i < zone_num; i++){
		all_zones[i].valid = 1;

		/* filter zone lower thatn working zone */
		if (all_zones[i].h  < working_zone)
			all_zones[i].valid = 0;
	}


	/* 
	   generate correct zone at ground level 
	   (depending on working zone)
	*/
	if (working_zone == 2)
		zone_num =  generate_center_ground_zones(buffer, x_in, y_in, 
							 all_zones, zone_num, max_zones, center_x, center_y);
	else
		zone_num =  generate_rectangle_ground_zones(buffer, x_in, y_in, 
							    all_zones, zone_num, max_zones, center_x, center_y,
							    working_zone);

	/* filter zone position, depending on workingzone */	
	if (working_zone == 2)
		zone_num = zone_filter_center(zone_num, all_zones, center_x, center_y, tweak_min_margin);
	else
		zone_num = zone_filter_zone_rect(zone_num, all_zones, center_x, center_y , working_zone);
		


	/* display zones (debug purpose) */

	for (i = 0; i < zone_num; i++){
		//buffer[all_zones[i].p.y*x_in+all_zones[i].p.x] = 0x3;
		IMG_NOTICE("h:%d (v:%d) x:%"PRIi32" y:%"PRIi32"", all_zones[i].h, all_zones[i].valid, all_zones[i].p.x, all_zones[i].p.y);
 
	}
  
	IMG_NOTICE("num zones: %d", zone_num);

	
	

	return zone_num;
}


/*
  return -1 if not column dropzone is found
  return column hight if found
*/
int8_t get_column_dropzone(unsigned char *buffer, int16_t x_in, int16_t y_in, 
			   uint8_t working_zone, int16_t center_x, int16_t center_y,
			   int16_t * dropzone_x, int16_t * dropzone_y)
{
	uint8_t zone_num;
	int c_drop_zone;



	zone_num = filter_zones(buffer, x_in, y_in,
				g_all_zones, g_zone_num, MAX_ZONES,
				working_zone, center_x, center_y,
				0);

	c_drop_zone = find_column_dropzone(zone_num, g_all_zones);

	if (c_drop_zone<0)
		return -1;

	*dropzone_x = g_all_zones[c_drop_zone].p.x;
	*dropzone_y = g_all_zones[c_drop_zone].p.y;

	*dropzone_x = (*dropzone_x) * PIXEL2CM;
	*dropzone_y = (*dropzone_y) * PIXEL2CM + 30;

	return g_all_zones[c_drop_zone].h;
}

#define ROBOT_SEMI_INTERCOLUMN_SPACE 75

uint8_t is_temple_there(unsigned char * buffer, int16_t x_in, int16_t y_in, 
			uint8_t h, int16_t center_x, int16_t center_y)
{
	zone z;
	int ret;
	int i;

	
	IMG_NOTICE("test z (mm) : x:%d y:%d", center_x, center_y);
	IMG_NOTICE("test z:(pix): x:%d y:%d", (int)(center_x/ PIXEL2CM), (int)(center_y/ PIXEL2CM));
	
	
	z.p.x = center_x / PIXEL2CM;
	z.p.y = (center_y - ROBOT_SEMI_INTERCOLUMN_SPACE) / PIXEL2CM ;
	z.h = h;
	z.valid = 1;
	ret = zone_has_enought_pixels(buffer, x_in, y_in, &z);
	IMG_NOTICE("test z1: %d", ret);

	if (!ret)
		return 0;

	z.p.x = center_x / PIXEL2CM;
	z.p.y = (center_y  + ROBOT_SEMI_INTERCOLUMN_SPACE)/ PIXEL2CM;
	z.h = h;
	z.valid = 1;
	ret = zone_has_enought_pixels(buffer, x_in, y_in, &z);
	IMG_NOTICE("test z2: %d", ret);
	if (!ret)
		return 0;


	/* 
	   if middle zone is less or egual to tested temple 
	 */
	z.p.x = center_x / PIXEL2CM;
	z.p.y = center_y / PIXEL2CM;
	
	for (i = h; i > 0; i--){
		z.h = i;
		z.valid = 1;
		ret = zone_has_enought_pixels(buffer, x_in, y_in, &z);
		IMG_NOTICE("test z3:(h=%d) %d", i, ret);
		
		if (ret)
			return 1;
	}

	return 0;

}


int8_t find_temple_dropzone(unsigned char *buffer, int16_t x_in, int16_t y_in, 
			    uint8_t working_zone, int16_t center_x, int16_t center_y,
			    int16_t * dropzone_x, int16_t * dropzone_y)
{
	uint8_t zone_num;
	int8_t sisters[MAX_ZONES][MAX_SISTER_PER_ZONE];
	int8_t z_n = -1;
	uint8_t i, j;


	/* find all drop zone */
	zone_num = filter_zones(buffer, x_in, y_in,
				g_all_zones, g_zone_num, MAX_ZONES,
				working_zone, center_x, center_y,
				-2);

	/* precompute possible twin towers */
	find_twin_tower(zone_num, g_all_zones, sisters, center_x, center_y);
	

 	for (i=0; i< zone_num; i++){
		IMG_DEBUG("all sisters: %d", i);
		for (j=0;j<MAX_SISTER_PER_ZONE;j++){
			IMG_DEBUG("s: %d", sisters[i][j]);
		}
	}

	/* only use first sister of each zone */
 	for (i=0; i< zone_num; i++){
		
		/* if zone doesn't have twin tower */
		if (sisters[i][0] == -1)
			continue;

		if (!g_all_zones[i].valid)
			continue;

		if (z_n == -1){
			z_n = i;
			continue;
		}

		/* if we found higher twin tower */
		if (g_all_zones[z_n].h > g_all_zones[i].h)
			continue;

		z_n = i;
	}

	IMG_NOTICE("twin tower found :z_n=%d", z_n);
	if (z_n ==  -1)
		return -1;
	
	IMG_NOTICE("(%"PRIi32" %"PRIi32") (%"PRIi32" %"PRIi32")", 
		   g_all_zones[z_n].p.x, g_all_zones[z_n].p.y,
		   g_all_zones[sisters[z_n][0]].p.x, g_all_zones[sisters[z_n][0]].p.y);
	

	*dropzone_x = ((double)(g_all_zones[z_n].p.x + g_all_zones[sisters[z_n][0]].p.x)*PIXEL2CM ) / 2;
	*dropzone_y = ((double)(g_all_zones[z_n].p.y + g_all_zones[sisters[z_n][0]].p.y)*PIXEL2CM ) / 2 + 30;


	return g_all_zones[z_n].h;

}
 
