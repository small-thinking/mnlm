" enable pathogen plugin
call pathogen#infect()
call pathogen#helptags()
filetype plugin indent on

"	set omnicomplete
set nocp
filetype plugin on
set omnifunc=syntaxcomplete#Complete


syntax enable 
set background=dark
colorscheme solarized

"set t_Co=256

" Basics {
set nocompatible        " turn off vi-compatible mode 
set noexrc              " don't use local version of .(g)vimrc, .exrc
set background=dark     " blue shows much better
" solarized options 
let g:solarized_visibility = "high"
let g:solarized_contrast = "high"
let g:solarized_termcolors=256
"colorscheme desert
colorscheme solarized
"         " }

" Vim UI {
"highlight StatusLineNC ctermbg=gray ctermfg=gray
" the following lines explicitly define status line coloring when splitting
"hi ModeMsg cterm=bold ctermbg=red
"hi StatusLine term=reverse ctermfg=2 ctermbg=white gui=bold,reverse
hi StatusLineNC term=reverse ctermfg=black ctermbg=darkyellow cterm=none
" hi StatusLineNC term=reverse cterm=none 
" highlight StatusLine   term=reverse ctermfg=LightGray ctermbg=Bluecterm=NONE
" highlight StatusLineNC term=reverse ctermfg=DarkCyan ctermbg=Blue cterm=NONE
"set cursorline
set mouse=a
set colorcolumn=80      " color the characters exceeds the column limit 
set incsearch           " highlight as search phrase is typed
set laststatus=2        " always show status line
set linespace=0         " don't insert extra pixel lines between rows
set listchars=tab:>-,trail:-  " show tabs and trailing
set nostartofline   " leave cursor in current position
set number        " turn on line numbers
set numberwidth=5   " 5 digits displayed for line numbers (0-99999)
set report=0      " report anything changed with :...
set ruler       " show current position along bottom
set scrolloff=10    " keep 10 lines (top/bottom) in view for scope
set showcmd       " show command being typed
set showmatch     " show matching brackets
set sidescrolloff=10  " keep 5 lines at size
set statusline=%F%m%r%h%w\ %=[C%v,L%l/%L]
"              | | | | |   |   |   |  + total lines 
"              | | | | |   |   |   + current line 
"              | | | | |   |   + current column 
"              | | | | |   + right align rest of status 
"              | | | | +-- preview flag in square brackets
"              | | | +-- help flag in square brackets
"              | | +-- readonly flag in square brackets
"              | +-- modified flag in square brackets
"              +-- full path to file in the buffer
" }


" set to autoread when a file is modified outside
set autoread