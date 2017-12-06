
ifneq ($V,1)
Q ?= @
endif

SRC	=	pins.tex	\
		odroid_xu3.tex	\
		odroid_c1.tex	\
		odroid_c2.tex	\
		odroid_n1.tex

DVI	=	$(SRC:.tex=.dvi)

all:	$(DVI)
	$Q echo	Generating DVI

.tex.dvi:
	$Q latex $< $@
	$Q dvipdf $@

.PHONY:	clean
clean:
	$Q rm -f *.dvi *.aux *.log *.ps *.toc *.bak *~
