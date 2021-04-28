\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{LTHtwocol}%
  [2018/05/05 A class for two column documents  at LTH, based on memoir]

\DeclareOption{final}{
  \AtBeginDocument{\pagestyle{empty}\overfullrule=0pt}
}
  

\newif\ifno@biblatex
\DeclareOption{biblatex=none}{%
  \no@biblatextrue
}

\newif\if@biblatex@numeric\@biblatex@numerictrue
\newcommand\biblatex@style{style=numeric}
\DeclareOption{biblatex=numeric}{
  \@biblatex@numerictrue
  \renewcommand\biblatex@style{style=numeric}
}
\DeclareOption{biblatex=authoryear}{
  \@biblatex@numericfalse
  \renewcommand\biblatex@style{style=authoryear}
}


\newcommand\bibtex@backend{biber}
\DeclareOption{biblatex=bibtex}{%
  \renewcommand\bibtex@backend{bibtex}
}
\DeclareOption{biblatex=biber}{%
 \renewcommand\bibtex@backend{biber}
}

\PassOptionsToClass{article,10pt}{memoir}%
\ProcessOptions
  
\LoadClass{memoir}
\RequirePackage{etoolbox}
\stockaiv\pageaiv
%\textheight=240mm%\textwidth=140mm
\setlrmarginsandblock{15mm}{15mm}{*}
\setulmarginsandblock{20mm}{30mm}{*}
\setcolsepandrule{8mm}{0pt}
\headheight=14pt\headsep=14pt
\footskip=12mm
\checkandfixthelayout

\renewcommand\thesection{\arabic{section}.}
\setsecnumdepth{subsection}
\renewcommand\thesubsection{\thesection\arabic{subsection}}
\renewcommand\appendix{\par
  \setcounter{section}{0}%
  \renewcommand\thesection{\@Alph\c@section.}%
  \anappendixtrue}

%%%%%%%%%%%%%%%% section style %%%%%%%%%%%%%%%%%%%
\setbeforesecskip{-12pt plus -6 pt minus -3pt}
\setsecheadstyle{\bfseries\Large\raggedright}
\setaftersecskip{6pt}
\def\sectionmark#1{\markright{\ifnum \c@secnumdepth > \z@
        \thesection \quad\fi #1}}
%%%%%%%%%%%%%% subsection style %%%%%%%%%%%%%%%%%%%%
\setbeforesubsecskip{-8pt plus -4pt minus -2pt}
\setsubsecheadstyle{\bfseries\raggedright}
\setaftersubsecskip{1pt}

%%%%%%%%%%%%% subsubsection style %%%%%%%%%%%%%%%%%%%%%%%%
\setbeforesubsubsecskip{6pt plus 2pt minus 1pt}
\setaftersubsubsecskip{-1em}
\setsubsubsecheadstyle{\normalsize\bfseries\itshape}

%%%%%%%%%%%%% subsubsection style %%%%%%%%%%%%%%%%%%%%%%%%
\setbeforesubsubsecskip{6pt plus 2pt minus 1pt}
\setaftersubsubsecskip{-1em}
\setsubsubsecheadstyle{\normalsize\bfseries\itshape}

%%%%%%%%%%% Section numbering %%%%%%%%%%
\setsecnumformat{\csname the#1\endcsname\quad}

%%%%%%%%%%%%%% Captions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\captiondelim{.\quad}
\captionnamefont{\small\bfseries}
\captiontitlefont{\small}
\captionstyle[\centering]{}

%%%%%%%%%%%
\newbox\fm@box
\newdimen\@frontmatterwidth
\newcount\sv@hyphenpenalty     
\@frontmatterwidth=0.8\textwidth


\newcommand*\fm@title@{An Important Contribution}

\renewenvironment{abstract}{%
  \lrbox{\abstract@box}\begin{minipage}{\@frontmatterwidth}
    \rule{\@frontmatterwidth}{0.5pt}
    \begin{adjustwidth}{4mm}{4mm}\noindent\textit{\textbf{Abstract:}}%
      \quad\ignorespaces
}{\par\end{adjustwidth}\medskip\hrule\end{minipage}\endlrbox}

\newsavebox{\abstract@box}
\def\title@font{\Huge\bfseries}
\newcounter{auth@num}

\renewenvironment{frontmatter}{%
  \begin{lrbox}{\fm@box}\begin{minipage}{\@frontmatterwidth}
  \renewcommand*\title[1]{\renewcommand*\fm@title@{##1}}
  \newcommand*\fm@authors@{\hspace*{-1em}}
  \renewcommand\author{\@ifnextchar[{\author@optarg}{\author@optarg[]}}
  \def\author@optarg[##1]##2{\ifblank{##1}%
    {\appto\fm@authors@{\quad\mbox{##2}}}%
    {\stepcounter{auth@num}\csedef{##1@auth@}{\arabic{auth@num}}%
      \appto\fm@authors@{\quad\mbox{##2$^{\mathrm{\csuse{##1@auth@}}}$}}%
    }% end \ifblank
  }% end \author@optarg
  \newcommand\advisor[1]{\providecommand*\fm@advisor@{##1}}
  \newcommand\email{\providecommand*\fm@email@{\hspace*{-1em}}% will only be done once
    \@ifnextchar[{\email@optarg}{\email@optarg[]}}
  \def\email@optarg[##1]##2{\message{##2} \ifblank{##1}%
    {\PackageError{LTHtwocol}{Email specification must have a key.}{}}%
    {\letcs\mail@key{##1@auth@}%
       \ifundef\mail@key{\PackageError{LTHtwocol}%
          {Email specification key [##1] invalid.}{}}{}%
     \appto\fm@email@{\quad\mbox{$^{\mathrm{\csuse{##1@auth@}}}$\texttt{##2}}}}%
  }
  \newcommand*\affiliation{\providecommand*\fm@affiliation@{\hspace*{-1em}}%
    \@ifnextchar[{\affiliation@optarg}{\affiliation@optarg[]}}
  \newcommand*\affiliation@optarg[2][]{\message{##2} \ifblank{##1}%
    {\PackageError{LTHtwocol}{Affiliation specification must have a key.}{}}%
    {\letcs\affil@key{##1@auth@}%
      \ifundef\affil@key{\PackageError{LTHtwocol}%
        {Affiliation specification key [##1] invalid}{extratext}}
      \appto\fm@affiliation@{\quad\mbox{$^{\mathrm{\csuse{##1@auth@}}}$\textit{##2}}}}}%
 }{% frontmatter
    \centering
    \vspace*{10mm}
    {\title@font\fm@title@\par}\vspace{4mm}%
    \noindent{\large\bfseries\fm@authors@\par}
    \ifdef\fm@advisor@{\vspace*{2mm}\noindent\normalsize
          Project Advisor: \textbf{\fm@advisor@}\par}{}
    \ifdef\fm@email@{\vspace*{2mm}{\noindent\small\fm@email@\par}}{}
    \ifdef\fm@affiliation@{\vspace*{2mm}\noindent\normalsize\fm@affiliation@\par}{}
    \vspace*{4mm}\usebox{\abstract@box}%
    \vspace*{10mm}
  \end{minipage}\end{lrbox}
  \twocolumn[\hbox to \textwidth{\hss\usebox{\fm@box}\hss}]
}% endfrontmatter


\usepackage{newtxtext,newtxmath}
\usepackage[varl]{inconsolata}
\frenchspacing\tolerance=1000\emergencystretch=1em
\flushbottom\overfullrule=6pt

%%%%%%%%%%%%% biblatex %%%%%%%%%%%%%%%%%%
\ifno@biblatex \endinput \fi
\renewcommand\memUChead[1]{#1}


\PassOptionsToPackage{\biblatex@style}{biblatex}
\PassOptionsToPackage{backend=\bibtex@backend}{biblatex}
\RequirePackage{biblatex}

\ExecuteBibliographyOptions{maxcitenames=2,maxbibnames=99}
\if@biblatex@numeric\relax\else
  \ExecuteBibliographyOptions{dashed=false}
\fi
\ExecuteBibliographyOptions{giveninits=true,uniquename=false,uniquelist=false}
\ExecuteBibliographyOptions{urldate=edtf,seconds=true}
\AtEveryBibitem{\clearfield{month}}

\let\cite=\parencite
\newcommand\mkciteparens[1]{{[#1]}}
\DeclareCiteCommand{\parencite}[\mkciteparens]%
  {\usebibmacro{prenote}}
  {\usebibmacro{citeindex}%
   \usebibmacro{cite}}
  {\multicitedelim}
  {\usebibmacro{postnote}}
\renewcommand*{\nameyeardelim}{\addcomma\addspace}
 
\renewcommand*{\bibsetup}{%
  \interlinepenalty=5000\relax
  \widowpenalty=10000\relax
  \clubpenalty=10000\relax
  \raggedbottom
  \frenchspacing
  \tolerance=2000
  \emergencystretch=2em
  \setlength{\bibitemsep}{\smallskipamount}
  \biburlsetup}


% We want to remove an 'in:' from the article entries. Unfortunately
% this requires a complete copy of the Driver.
\DeclareBibliographyDriver{article}{%
  \usebibmacro{bibindex}%
  \usebibmacro{begentry}%
  \usebibmacro{author/translator+others}%
  \setunit{\labelnamepunct}\newblock
  \usebibmacro{title}%
  \newunit
  \printlist{language}%
  \newunit\newblock
  \usebibmacro{byauthor}%
  \newunit\newblock
  \usebibmacro{bytranslator+others}%
  \newunit\newblock
  \printfield{version}%
  \newunit\newblock
  %\usebibmacro{in:}%
  \usebibmacro{journal+issuetitle}%
  \newunit
  \usebibmacro{byeditor+others}%
  \newunit
  \usebibmacro{note+pages}%
  \newunit\newblock
  \iftoggle{bbx:isbn}
    {\printfield{issn}}
    {}%
  \newunit\newblock
  \usebibmacro{doi+eprint+url}%
  \newunit\newblock
  \usebibmacro{addendum+pubstate}%
  \setunit{\bibpagerefpunct}\newblock
  \usebibmacro{pageref}%
  \newunit\newblock
  \usebibmacro{related}%
  \usebibmacro{finentry}}

% We want to include a number for the theses. Unfortunately this
% requires a complete copy of the driver.
\DeclareBibliographyDriver{thesis}{%
  \usebibmacro{bibindex}%
  \usebibmacro{begentry}%
  \usebibmacro{author}%
  \setunit{\labelnamepunct}\newblock
  \usebibmacro{title}%
  \newunit
  \printlist{language}%
  \newunit\newblock
  \usebibmacro{byauthor}%
  \newunit\newblock
  \printfield{note}%
  \newunit\newblock
  \printfield{type}%
  \setunit*{\addspace}%
  \printfield{number}% Added locally
  \newunit
  \usebibmacro{institution+location+date}%
  \newunit\newblock
  \usebibmacro{chapter+pages}%
  \newunit
  \printfield{pagetotal}%
  \newunit\newblock
  \iftoggle{bbx:isbn}
    {\printfield{isbn}}
    {}%
  \newunit\newblock
  \usebibmacro{doi+eprint+url}%
  \newunit\newblock
  \usebibmacro{addendum+pubstate}%
  \setunit{\bibpagerefpunct}\newblock
  \usebibmacro{pageref}%
  \newunit\newblock
  \usebibmacro{related}%
  \usebibmacro{finentry}}

% Make theses look like book titles
\DeclareFieldFormat[thesis]{title}{\mkbibemph{#1}}

% The original version has the form vol.num.
% we want <bold>vol</bold>:num
\renewbibmacro*{volume+number+eid}{%
  \printfield[bold]{volume}%
  \setunit*{\addcolon}%
  \printfield{number}%
  \setunit{\addcomma\space}%
  \printfield{eid}}

% The original version had the form address: publisher.
% We want publisher, address.
\renewbibmacro*{publisher+location+date}{%
  \printlist{publisher}%
  \setunit*{\addcomma\space}%
  \printlist{location}%
  \setunit*{\addcomma\space}%
  \usebibmacro{date}%
  \newunit}

% The original version had the form address: institution.
% We want institution, address.
\renewbibmacro*{institution+location+date}{%
  \printlist{institution}
  \setunit*{\addcomma\space}%
  \printlist{location}%
  \setunit*{\addcomma\space}%
  \usebibmacro{date}%
  \newunit}

% Make the titles of articles and conference contributions 'sentence case',
% but don't touch any other titles.
% http://tex.stackexchange.com/questions/22980/sentence-case-for-titles-in-biblatex
\DeclareFieldFormat{titlecase}{\MakeTitleCase{#1}}
\newrobustcmd{\MakeTitleCase}[1]{%
  \ifthenelse{\ifcurrentfield{booktitle}\OR\ifcurrentfield{booksubtitle}%
    \OR\ifcurrentfield{maintitle}\OR\ifcurrentfield{mainsubtitle}%
    \OR\ifcurrentfield{journaltitle}\OR\ifcurrentfield{journalsubtitle}%
    \OR\ifcurrentfield{issuetitle}\OR\ifcurrentfield{issuesubtitle}%
    \OR\ifentrytype{book}\OR\ifentrytype{mvbook}\OR\ifentrytype{bookinbook}%
    \OR\ifentrytype{booklet}\OR\ifentrytype{suppbook}%
    \OR\ifentrytype{collection}\OR\ifentrytype{mvcollection}%
    \OR\ifentrytype{suppcollection}\OR\ifentrytype{manual}%
    \OR\ifentrytype{periodical}\OR\ifentrytype{suppperiodical}%
    \OR\ifentrytype{proceedings}\OR\ifentrytype{mvproceedings}%
    \OR\ifentrytype{reference}\OR\ifentrytype{mvreference}%
    \OR\ifentrytype{report}\OR\ifentrytype{thesis}}
    {#1}
    {\MakeSentenceCase{#1}}}

% We want incollection and inbook entries to have editor specifications like:
% In: Editor E. (Ed.). Booktitle. Publisher. Address
%
\newbibmacro*{ineditor+others}{%
  \ifboolexpr{ test \ifuseeditor and not test {\ifnameundef{editor}} }
    {\defcounter{maxnames}{1}% Only show one editor, use et al. if more.
      \printnames{editor}%
     \setunit{\addspace}%
     \usebibmacro{editor+othersstrg}%
     \clearname{editor}}
    {}}

% Print editors before "in" title
\renewbibmacro{in:}{%
  \ifentrytype{article}{}{\printtext{\bibstring{in}\intitlepunct}}%
  \usebibmacro{ineditor+others}%
  \newunit
  \clearname{editor}}

\DeclareFieldFormat{editortype}{\mkbibparens{#1}}

\DefineBibliographyStrings{english}{% various adjustments to
                                    % common bib entry strings
  editor = {Ed.},
  editors = {Eds.},
}
