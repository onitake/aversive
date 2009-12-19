typedef struct _vect_t {
  int32_t x;
  int32_t y;
}vect_t;

typedef struct _point_t {
  int32_t x;
  int32_t y;
}point_t;


 
/* Return scalar product */
int32_t 
vect_pscal(vect_t *v, vect_t *w);

/* Return Z of vectorial product */
int32_t 
vect_pvect(vect_t *v, vect_t *w);

/* Return scalar product */
int8_t 
vect_pscal_sign(vect_t *v, vect_t *w);

/* Return Z of vectorial product */
int8_t 
vect_pvect_sign(vect_t *v, vect_t *w);

/* norm of a vector */
float vect_norm(vect_t *v);
void vect_rot_trigo(vect_t *v);
void vect_rot_retro(vect_t *v);
float vect_get_angle(vect_t *v, vect_t *w);
  
