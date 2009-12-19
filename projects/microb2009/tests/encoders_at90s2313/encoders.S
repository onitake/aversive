	;; PIND = X X X SEL A1 B1 A2 B2
	
	;; r2 -> codeur 1
	;; r3 -> codeur 2

	;; r17 -> codAB
	;; r5 -> codAB_prec
	;; r18 -> 0
	
.global	main

	.type	main, @function
main:
	;; DDRB = 0xFF, en sortie
	ldi r24,lo8(-1)
	out 55-0x20,r24

	;; on initialise à 0 les registres
	ldi r17,0
	ldi r18,0
	ldi r30,0
	ldi r31,1
	mov r5,r30
	mov r2,r30
	mov r3,r30

	
	;; Etapes :
	;; Lit le port
	;; Saute à l'adresse PORT * 2 + offset
	;; Saute a la routine d'incrément qui va bien
	;; Met a jour la sortie
	;; Retourne au début
boucle:
	;; 1 cycle
	out 56-0x20,r2

	;; r5 va contenir l'etat precedent de codAB, décalé de 4 vers la gauche
	;; 1 cycle
	mov r5,r17
	;; 1 cycle
	swap r5
		
	;; Lit le pinD (les 4 LSB) dans r17
	;; 1 cycle
	in r17,48-0x20
	;; 1 cycle
	cbr r17,0xF0

	;; on le place dans r0
	;; 1 cycle
	mov r0, r17

	;; on y additionne l'etat precedent (r5)
	;; 1 cycle
	add r0,r5
	
	;; On met tout ça dans Z (r30/r31)
	;; 1 cycle
	mov r30,r0

	;; On saute à l'adresse du tableau
	;; 2 cycles
	ijmp


	
moins_moins:	
	;; 1 cycle
	dec r2
	;; 2 cycles
	rjmp boucle
moins_zero:
	dec r2
	rjmp boucle
moins_plus:
	dec r2
	rjmp boucle

zero_moins:
	rjmp boucle
zero_zero:
	rjmp boucle
zero_plus:
	rjmp boucle

plus_moins:
	inc r2
	rjmp boucle
plus_zero:
	inc r2
	rjmp boucle
plus_plus:
	inc r2
	rjmp boucle



; lecture_sel:	
; 	;; Lit le pinD dans r6 pour récupérer le bit SEL
; 	in r6,48-0x20

; 	;; Passe l'instruction suivante si le bit de sel est 0
; 	;; (skip if bit in register is cleared)
; 	sbrc r6,4
; 	rjmp cpt2

; cpt1:
; 	out 56-0x20,r2
; 	rjmp boucle

; cpt2:	
; 	out 56-0x20,r3
; 	rjmp boucle



.org 0x1DE
tableau:

	;; VAL =   0 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   0   0
	;; 2 cycles
	rjmp zero_zero

	;; VAL =   1 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   0   1
	rjmp zero_plus

	;; VAL =   2 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   0   3
	rjmp zero_moins

	;; VAL =   3 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   0   2
	rjmp zero_zero

	;; VAL =   4 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   1   0
	rjmp plus_zero

	;; VAL =   5 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   1   1
	rjmp plus_plus

	;; VAL =   6 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   1   3
	rjmp plus_moins

	;; VAL =   7 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   1   2
	rjmp plus_zero

	;; VAL =   8 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   3   0
	rjmp moins_zero

	;; VAL =   9 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   3   1
	rjmp moins_plus

	;; VAL =  10 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   3   3
	rjmp moins_moins

	;; VAL =  11 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   3   2
	rjmp moins_zero

	;; VAL =  12 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   2   0
	rjmp zero_zero

	;; VAL =  13 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   2   1
	rjmp zero_plus

	;; VAL =  14 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   2   3
	rjmp zero_moins

	;; VAL =  15 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   0   2   2
	rjmp zero_zero

	;; VAL =  16 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   0   0
	rjmp zero_moins

	;; VAL =  17 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   0   1
	rjmp zero_zero

	;; VAL =  18 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   0   3
	rjmp zero_zero

	;; VAL =  19 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   0   2
	rjmp zero_plus

	;; VAL =  20 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   1   0
	rjmp plus_moins

	;; VAL =  21 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   1   1
	rjmp plus_zero

	;; VAL =  22 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   1   3
	rjmp plus_zero

	;; VAL =  23 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   1   2
	rjmp plus_plus

	;; VAL =  24 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   3   0
	rjmp moins_moins

	;; VAL =  25 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   3   1
	rjmp moins_zero

	;; VAL =  26 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   3   3
	rjmp moins_zero

	;; VAL =  27 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   3   2
	rjmp moins_plus

	;; VAL =  28 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   2   0
	rjmp zero_moins

	;; VAL =  29 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   2   1
	rjmp zero_zero

	;; VAL =  30 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   2   3
	rjmp zero_zero

	;; VAL =  31 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   1   2   2
	rjmp zero_plus

	;; VAL =  32 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   0   0
	rjmp zero_plus

	;; VAL =  33 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   0   1
	rjmp zero_zero

	;; VAL =  34 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   0   3
	rjmp zero_zero

	;; VAL =  35 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   0   2
	rjmp zero_moins

	;; VAL =  36 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   1   0
	rjmp plus_plus

	;; VAL =  37 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   1   1
	rjmp plus_zero

	;; VAL =  38 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   1   3
	rjmp plus_zero

	;; VAL =  39 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   1   2
	rjmp plus_moins

	;; VAL =  40 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   3   0
	rjmp moins_plus

	;; VAL =  41 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   3   1
	rjmp moins_zero

	;; VAL =  42 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   3   3
	rjmp moins_zero

	;; VAL =  43 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   3   2
	rjmp moins_moins

	;; VAL =  44 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   2   0
	rjmp zero_plus

	;; VAL =  45 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   2   1
	rjmp zero_zero

	;; VAL =  46 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   2   3
	rjmp zero_zero

	;; VAL =  47 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   3   2   2
	rjmp zero_moins

	;; VAL =  48 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   0   0
	rjmp zero_zero

	;; VAL =  49 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   0   1
	rjmp zero_moins

	;; VAL =  50 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   0   3
	rjmp zero_plus

	;; VAL =  51 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   0   2
	rjmp zero_zero

	;; VAL =  52 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   1   0
	rjmp plus_zero

	;; VAL =  53 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   1   1
	rjmp plus_moins

	;; VAL =  54 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   1   3
	rjmp plus_plus

	;; VAL =  55 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   1   2
	rjmp plus_zero

	;; VAL =  56 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   3   0
	rjmp moins_zero

	;; VAL =  57 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   3   1
	rjmp moins_moins

	;; VAL =  58 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   3   3
	rjmp moins_plus

	;; VAL =  59 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   3   2
	rjmp moins_zero

	;; VAL =  60 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   2   0
	rjmp zero_zero

	;; VAL =  61 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   2   1
	rjmp zero_moins

	;; VAL =  62 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   2   3
	rjmp zero_plus

	;; VAL =  63 -  C1(t-1) C2(t-1) C1 C2 (naturel)   0   2   2   2
	rjmp zero_zero

	;; VAL =  64 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   0   0
	rjmp moins_zero

	;; VAL =  65 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   0   1
	rjmp moins_plus

	;; VAL =  66 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   0   3
	rjmp moins_moins

	;; VAL =  67 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   0   2
	rjmp moins_zero

	;; VAL =  68 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   1   0
	rjmp zero_zero

	;; VAL =  69 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   1   1
	rjmp zero_plus

	;; VAL =  70 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   1   3
	rjmp zero_moins

	;; VAL =  71 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   1   2
	rjmp zero_zero

	;; VAL =  72 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   3   0
	rjmp zero_zero

	;; VAL =  73 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   3   1
	rjmp zero_plus

	;; VAL =  74 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   3   3
	rjmp zero_moins

	;; VAL =  75 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   3   2
	rjmp zero_zero

	;; VAL =  76 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   2   0
	rjmp plus_zero

	;; VAL =  77 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   2   1
	rjmp plus_plus

	;; VAL =  78 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   2   3
	rjmp plus_moins

	;; VAL =  79 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   0   2   2
	rjmp plus_zero

	;; VAL =  80 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   0   0
	rjmp moins_moins

	;; VAL =  81 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   0   1
	rjmp moins_zero

	;; VAL =  82 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   0   3
	rjmp moins_zero

	;; VAL =  83 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   0   2
	rjmp moins_plus

	;; VAL =  84 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   1   0
	rjmp zero_moins

	;; VAL =  85 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   1   1
	rjmp zero_zero

	;; VAL =  86 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   1   3
	rjmp zero_zero

	;; VAL =  87 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   1   2
	rjmp zero_plus

	;; VAL =  88 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   3   0
	rjmp zero_moins

	;; VAL =  89 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   3   1
	rjmp zero_zero

	;; VAL =  90 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   3   3
	rjmp zero_zero

	;; VAL =  91 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   3   2
	rjmp zero_plus

	;; VAL =  92 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   2   0
	rjmp plus_moins

	;; VAL =  93 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   2   1
	rjmp plus_zero

	;; VAL =  94 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   2   3
	rjmp plus_zero

	;; VAL =  95 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   1   2   2
	rjmp plus_plus

	;; VAL =  96 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   0   0
	rjmp moins_plus

	;; VAL =  97 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   0   1
	rjmp moins_zero

	;; VAL =  98 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   0   3
	rjmp moins_zero

	;; VAL =  99 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   0   2
	rjmp moins_moins

	;; VAL = 100 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   1   0
	rjmp zero_plus

	;; VAL = 101 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   1   1
	rjmp zero_zero

	;; VAL = 102 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   1   3
	rjmp zero_zero

	;; VAL = 103 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   1   2
	rjmp zero_moins

	;; VAL = 104 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   3   0
	rjmp zero_plus

	;; VAL = 105 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   3   1
	rjmp zero_zero

	;; VAL = 106 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   3   3
	rjmp zero_zero

	;; VAL = 107 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   3   2
	rjmp zero_moins

	;; VAL = 108 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   2   0
	rjmp plus_plus

	;; VAL = 109 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   2   1
	rjmp plus_zero

	;; VAL = 110 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   2   3
	rjmp plus_zero

	;; VAL = 111 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   3   2   2
	rjmp plus_moins

	;; VAL = 112 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   0   0
	rjmp moins_zero

	;; VAL = 113 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   0   1
	rjmp moins_moins

	;; VAL = 114 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   0   3
	rjmp moins_plus

	;; VAL = 115 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   0   2
	rjmp moins_zero

	;; VAL = 116 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   1   0
	rjmp zero_zero

	;; VAL = 117 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   1   1
	rjmp zero_moins

	;; VAL = 118 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   1   3
	rjmp zero_plus

	;; VAL = 119 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   1   2
	rjmp zero_zero

	;; VAL = 120 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   3   0
	rjmp zero_zero

	;; VAL = 121 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   3   1
	rjmp zero_moins

	;; VAL = 122 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   3   3
	rjmp zero_plus

	;; VAL = 123 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   3   2
	rjmp zero_zero

	;; VAL = 124 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   2   0
	rjmp plus_zero

	;; VAL = 125 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   2   1
	rjmp plus_moins

	;; VAL = 126 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   2   3
	rjmp plus_plus

	;; VAL = 127 -  C1(t-1) C2(t-1) C1 C2 (naturel)   1   2   2   2
	rjmp plus_zero

	;; VAL = 128 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   0   0
	rjmp plus_zero

	;; VAL = 129 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   0   1
	rjmp plus_plus

	;; VAL = 130 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   0   3
	rjmp plus_moins

	;; VAL = 131 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   0   2
	rjmp plus_zero

	;; VAL = 132 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   1   0
	rjmp zero_zero

	;; VAL = 133 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   1   1
	rjmp zero_plus

	;; VAL = 134 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   1   3
	rjmp zero_moins

	;; VAL = 135 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   1   2
	rjmp zero_zero

	;; VAL = 136 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   3   0
	rjmp zero_zero

	;; VAL = 137 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   3   1
	rjmp zero_plus

	;; VAL = 138 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   3   3
	rjmp zero_moins

	;; VAL = 139 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   3   2
	rjmp zero_zero

	;; VAL = 140 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   2   0
	rjmp moins_zero

	;; VAL = 141 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   2   1
	rjmp moins_plus

	;; VAL = 142 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   2   3
	rjmp moins_moins

	;; VAL = 143 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   0   2   2
	rjmp moins_zero

	;; VAL = 144 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   0   0
	rjmp plus_moins

	;; VAL = 145 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   0   1
	rjmp plus_zero

	;; VAL = 146 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   0   3
	rjmp plus_zero

	;; VAL = 147 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   0   2
	rjmp plus_plus

	;; VAL = 148 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   1   0
	rjmp zero_moins

	;; VAL = 149 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   1   1
	rjmp zero_zero

	;; VAL = 150 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   1   3
	rjmp zero_zero

	;; VAL = 151 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   1   2
	rjmp zero_plus

	;; VAL = 152 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   3   0
	rjmp zero_moins

	;; VAL = 153 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   3   1
	rjmp zero_zero

	;; VAL = 154 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   3   3
	rjmp zero_zero

	;; VAL = 155 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   3   2
	rjmp zero_plus

	;; VAL = 156 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   2   0
	rjmp moins_moins

	;; VAL = 157 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   2   1
	rjmp moins_zero

	;; VAL = 158 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   2   3
	rjmp moins_zero

	;; VAL = 159 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   1   2   2
	rjmp moins_plus

	;; VAL = 160 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   0   0
	rjmp plus_plus

	;; VAL = 161 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   0   1
	rjmp plus_zero

	;; VAL = 162 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   0   3
	rjmp plus_zero

	;; VAL = 163 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   0   2
	rjmp plus_moins

	;; VAL = 164 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   1   0
	rjmp zero_plus

	;; VAL = 165 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   1   1
	rjmp zero_zero

	;; VAL = 166 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   1   3
	rjmp zero_zero

	;; VAL = 167 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   1   2
	rjmp zero_moins

	;; VAL = 168 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   3   0
	rjmp zero_plus

	;; VAL = 169 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   3   1
	rjmp zero_zero

	;; VAL = 170 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   3   3
	rjmp zero_zero

	;; VAL = 171 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   3   2
	rjmp zero_moins

	;; VAL = 172 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   2   0
	rjmp moins_plus

	;; VAL = 173 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   2   1
	rjmp moins_zero

	;; VAL = 174 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   2   3
	rjmp moins_zero

	;; VAL = 175 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   3   2   2
	rjmp moins_moins

	;; VAL = 176 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   0   0
	rjmp plus_zero

	;; VAL = 177 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   0   1
	rjmp plus_moins

	;; VAL = 178 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   0   3
	rjmp plus_plus

	;; VAL = 179 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   0   2
	rjmp plus_zero

	;; VAL = 180 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   1   0
	rjmp zero_zero

	;; VAL = 181 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   1   1
	rjmp zero_moins

	;; VAL = 182 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   1   3
	rjmp zero_plus

	;; VAL = 183 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   1   2
	rjmp zero_zero

	;; VAL = 184 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   3   0
	rjmp zero_zero

	;; VAL = 185 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   3   1
	rjmp zero_moins

	;; VAL = 186 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   3   3
	rjmp zero_plus

	;; VAL = 187 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   3   2
	rjmp zero_zero

	;; VAL = 188 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   2   0
	rjmp moins_zero

	;; VAL = 189 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   2   1
	rjmp moins_moins

	;; VAL = 190 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   2   3
	rjmp moins_plus

	;; VAL = 191 -  C1(t-1) C2(t-1) C1 C2 (naturel)   3   2   2   2
	rjmp moins_zero

	;; VAL = 192 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   0   0
	rjmp zero_zero

	;; VAL = 193 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   0   1
	rjmp zero_plus

	;; VAL = 194 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   0   3
	rjmp zero_moins

	;; VAL = 195 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   0   2
	rjmp zero_zero

	;; VAL = 196 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   1   0
	rjmp moins_zero

	;; VAL = 197 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   1   1
	rjmp moins_plus

	;; VAL = 198 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   1   3
	rjmp moins_moins

	;; VAL = 199 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   1   2
	rjmp moins_zero

	;; VAL = 200 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   3   0
	rjmp plus_zero

	;; VAL = 201 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   3   1
	rjmp plus_plus

	;; VAL = 202 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   3   3
	rjmp plus_moins

	;; VAL = 203 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   3   2
	rjmp plus_zero

	;; VAL = 204 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   2   0
	rjmp zero_zero

	;; VAL = 205 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   2   1
	rjmp zero_plus

	;; VAL = 206 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   2   3
	rjmp zero_moins

	;; VAL = 207 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   0   2   2
	rjmp zero_zero

	;; VAL = 208 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   0   0
	rjmp zero_moins

	;; VAL = 209 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   0   1
	rjmp zero_zero

	;; VAL = 210 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   0   3
	rjmp zero_zero

	;; VAL = 211 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   0   2
	rjmp zero_plus

	;; VAL = 212 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   1   0
	rjmp moins_moins

	;; VAL = 213 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   1   1
	rjmp moins_zero

	;; VAL = 214 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   1   3
	rjmp moins_zero

	;; VAL = 215 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   1   2
	rjmp moins_plus

	;; VAL = 216 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   3   0
	rjmp plus_moins

	;; VAL = 217 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   3   1
	rjmp plus_zero

	;; VAL = 218 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   3   3
	rjmp plus_zero

	;; VAL = 219 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   3   2
	rjmp plus_plus

	;; VAL = 220 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   2   0
	rjmp zero_moins

	;; VAL = 221 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   2   1
	rjmp zero_zero

	;; VAL = 222 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   2   3
	rjmp zero_zero

	;; VAL = 223 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   1   2   2
	rjmp zero_plus

	;; VAL = 224 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   0   0
	rjmp zero_plus

	;; VAL = 225 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   0   1
	rjmp zero_zero

	;; VAL = 226 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   0   3
	rjmp zero_zero

	;; VAL = 227 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   0   2
	rjmp zero_moins

	;; VAL = 228 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   1   0
	rjmp moins_plus

	;; VAL = 229 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   1   1
	rjmp moins_zero

	;; VAL = 230 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   1   3
	rjmp moins_zero

	;; VAL = 231 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   1   2
	rjmp moins_moins

	;; VAL = 232 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   3   0
	rjmp plus_plus

	;; VAL = 233 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   3   1
	rjmp plus_zero

	;; VAL = 234 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   3   3
	rjmp plus_zero

	;; VAL = 235 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   3   2
	rjmp plus_moins

	;; VAL = 236 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   2   0
	rjmp zero_plus

	;; VAL = 237 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   2   1
	rjmp zero_zero

	;; VAL = 238 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   2   3
	rjmp zero_zero

	;; VAL = 239 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   3   2   2
	rjmp zero_moins

	;; VAL = 240 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   0   0
	rjmp zero_zero

	;; VAL = 241 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   0   1
	rjmp zero_moins

	;; VAL = 242 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   0   3
	rjmp zero_plus

	;; VAL = 243 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   0   2
	rjmp zero_zero

	;; VAL = 244 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   1   0
	rjmp moins_zero

	;; VAL = 245 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   1   1
	rjmp moins_moins

	;; VAL = 246 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   1   3
	rjmp moins_plus

	;; VAL = 247 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   1   2
	rjmp moins_zero

	;; VAL = 248 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   3   0
	rjmp plus_zero

	;; VAL = 249 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   3   1
	rjmp plus_moins

	;; VAL = 250 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   3   3
	rjmp plus_plus

	;; VAL = 251 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   3   2
	rjmp plus_zero

	;; VAL = 252 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   2   0
	rjmp zero_zero

	;; VAL = 253 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   2   1
	rjmp zero_moins

	;; VAL = 254 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   2   3
	rjmp zero_plus

	;; VAL = 255 -  C1(t-1) C2(t-1) C1 C2 (naturel)   2   2   2   2
	rjmp zero_zero
	

	
	

