$show_time = 1;
#$pdf_mode = 1; #For pdflatex
$pdf_mode = 4; #For lualatex
$postscript_mode = $dvi_mode = 0;
$lualatex = "lualatex --shell-escape %O %S";
$pdflatex = "pdflatex --shell-escape %O %S";
$out_dir = 'build';
system("mkdir -p build/build");
#$pdf_previewer = 'open -a preview'; # for mac
$pdf_previewer = "start atril";
$preview_mode = 1;
