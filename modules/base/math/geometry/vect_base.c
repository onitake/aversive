#include <stdint.h>
#include <math.h>
#include <vect_base.h>
 
/* Return scalar product */
int32_t 
vect_pscal(vect_t *v, vect_t *w)
{
	return v->x * w->x + v->y * w->y;
}

/* Return Z of vectorial product */
int32_t 
vect_pvect(vect_t *v, vect_t *w)
{
	return v->x*w->y - v->y*w->x;
}

/* Return scalar product */
int8_t 
vect_pscal_sign(vect_t *v, vect_t *w)
{
	int32_t z;
	z = vect_pscal(v, w);
	if (z==0)
		return 0;
	return z>0?1:-1;
}

/* Return Z of vectorial product */
int8_t 
vect_pvect_sign(vect_t *v, vect_t *w)
{
	int32_t z;
	z = vect_pvect(v, w);
	if (z==0)
		return 0;
	return z>0?1:-1;
}

/* norm of a vector */
float
vect_norm(vect_t *v)
{
	return sqrt(v->x*v->x+v->y*v->y);
}



void vect_rot_trigo(vect_t *v)
{
	int32_t s;
    
	s = v->x;
	v->x= -v->y;
	v->y = s;
}    

void vect_rot_retro(vect_t *v)
{
	int32_t s;
    
	s = v->x;
	v->x= v->y;
	v->y = -s;
}    


float vect_get_angle(vect_t *v, vect_t *w)
{
	int32_t ps;
	float n;
	
	ps = vect_pscal(v, w);
	n = vect_norm(v) * vect_norm(w);
	
	return acos((float)ps/n);
}
