LTX=latex
DVI=dvips
PDF=ps2pdf
DOC=mini_ref

mini_ref:	$(DOC).tex
	$(LTX) $(DOC).tex >& $(DOC).ltxlog
	$(LTX) $(DOC).tex >& $(DOC).ltxlog
	$(DVI) $(DOC).dvi -o $(DOC).ps >& $(DOC).dvilog
	$(PDF) $(DOC).ps > $(DOC).pdf

clean:
	rm -f $(DOC).aux $(DOC).dvi $(DOC).ps $(DOC).ltxlog $(DOC).dvilog $(DOC).log

