def gray2nat(x):
    if (x==3):return 2
    if (x==2):return 3
    return x

# C1(t-1) C2(t-1) C1 C2
for i in range(256):
    cod1 = (i&0x0C)>>2
    cod2 = (i&0x03)>>0
    cod1_prec = (i&0xC0)>>6
    cod2_prec = (i&0x30)>>4

    inc1 = gray2nat(cod1) - gray2nat(cod1_prec)
    if inc1==3: texte1= "moins"
    if inc1==2: texte1= "zero"
    if inc1==1: texte1= "plus"
    if inc1==0: texte1= "zero"
    if inc1==-1: texte1= "moins"
    if inc1==-2: texte1= "zero"
    if inc1==-3: texte1= "plus"

    inc2 = gray2nat(cod2) - gray2nat(cod2_prec)
    if inc2==3: texte2= "moins"
    if inc2==2: texte2= "zero"
    if inc2==1: texte2= "plus"
    if inc2==0: texte2= "zero"
    if inc2==-1: texte2= "moins"
    if inc2==-2: texte2= "zero"
    if inc2==-3: texte2= "plus"

                                                     
    print
    print "	;; VAL = %3d -  C1(t-1) C2(t-1) C1 C2 (naturel) %3d %3d %3d %3d"%(i,gray2nat(cod1_prec),gray2nat(cod2_prec),
                                           gray2nat(cod1),gray2nat(cod2))
    print "	rjmp %s_%s"%(texte1,texte2)
