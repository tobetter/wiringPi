dist_pdf_DATA = \
	odroid_c1.pdf \
	odroid_c2.pdf \
	odroid_c4.pdf \
	odroid_m1.pdf \
	odroid_n1.pdf \
	odroid_n2.pdf \
	odroid_xu3.pdf

.tex.pdf:
	pdflatex $< $@

clean-latex:
	rm -f *.pdf *.aux *.log

clean-local: clean-latex
