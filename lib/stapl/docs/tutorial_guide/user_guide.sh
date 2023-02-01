#
rm -f user_guide.aux user_guide.dvi user_guide.log user_guide.ps user_guide.pdf

cat ex_pre ex_200.cc ex_post > ex_200.tex
cat ex_pre ex_201.cc ex_post > ex_201.tex
cat ex_pre ex_202.cc ex_post > ex_202.tex
cat ex_pre ex_203.cc ex_post > ex_203.tex
cat ex_pre ex_204.cc ex_post > ex_204.tex
cat ex_pre ex_205.cc ex_post > ex_205.tex
cat ex_pre ex_206.cc ex_post > ex_206.tex
cat ex_pre ex_207.cc ex_post > ex_207.tex
cat ex_pre ex_208.cc ex_post > ex_208.tex
cat ex_pre ex_209.cc ex_post > ex_209.tex
cat ex_pre ex_210.cc ex_post > ex_210.tex
cat ex_pre ex_211.cc ex_post > ex_211.tex
cat ex_pre ex_212.cc ex_post > ex_212.tex

cat ex_pre ch3.hpp ex_post > ch3.tex

cat ex_pre ex_301.cc ex_post > ex_301.tex
cat ex_pre ex_302.cc ex_post > ex_302.tex
cat ex_pre ex_303.cc ex_post > ex_303.tex
cat ex_pre ex_304.cc ex_post > ex_304.tex
cat ex_pre ex_305.cc ex_post > ex_305.tex
cat ex_pre ex_306.cc ex_post > ex_306.tex

cat ex_pre ch4.hpp ex_post > ch4.tex

cat ex_pre ex_401.cc ex_post > ex_401.tex
cat ex_pre ex_402.cc ex_post > ex_402.tex
cat ex_pre ex_403.cc ex_post > ex_403.tex
cat ex_pre ex_404.cc ex_post > ex_404.tex
cat ex_pre ex_405.cc ex_post > ex_405.tex
cat ex_pre ex_406.cc ex_post > ex_406.tex

cat ex_pre ch5.hpp ex_post > ch5.tex

cat ex_pre ex_501.cc ex_post > ex_501.tex
cat ex_pre ex_502.cc ex_post > ex_502.tex
cat ex_pre ex_503.cc ex_post > ex_503.tex
cat ex_pre ex_504.cc ex_post > ex_504.tex
cat ex_pre ex_505.cc ex_post > ex_505.tex
cat ex_pre ex_506.cc ex_post > ex_506.tex
cat ex_pre ex_507.cc ex_post > ex_507.tex
cat ex_pre ex_508.cc ex_post > ex_508.tex

cat ex_pre ch6.hpp ex_post > ch6.tex

cat ex_pre ex_601.cc ex_post > ex_601.tex
cat ex_pre ex_602.cc ex_post > ex_602.tex
cat ex_pre ex_603.cc ex_post > ex_603.tex
cat ex_pre ex_604.cc ex_post > ex_604.tex
cat ex_pre ex_605.cc ex_post > ex_605.tex
cat ex_pre ex_606.cc ex_post > ex_606.tex
cat ex_pre ex_607.cc ex_post > ex_607.tex

#latex user_guide.tex
#exit

latex user_guide.tex >& user_guide1.log
latex user_guide.tex >& user_guide2.log
dvips user_guide.dvi -o user_guide.ps >& user_guide3.log
ps2pdf user_guide.ps > user_guide.pdf

rm -f ex_2??.tex ex_3??.tex ex_4??.tex ex_5??.tex ex_6??.tex ch3.tex ch4.tex ch5.tex ch6.tex
