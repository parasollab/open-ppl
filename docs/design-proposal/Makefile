TEX=PMPL-design-proposal

default_target: pdf

latex:
	latex $(TEX).tex
	latex $(TEX).tex

ps: latex
	dvips $(TEX).dvi -o $(TEX).ps
	
pdf: ps
	ps2pdf $(TEX).ps
	
clean:
	rm -f $(TEX).aux $(TEX).dvi $(TEX).log $(TEX).pdf $(TEX).ps $(TEX).toc

rmtrash:
	rm -f $(TEX).aux $(TEX).dvi $(TEX).log $(TEX).ps $(TEX).toc