# README

The following is copied from git@gitlab.com:parasol-lab/vim-stapl.git.
For access to the plugin, join the parasol-lab group on GitLab. There is more documentation
available there in the `doc/` directory, which can be accessed inside of vim
by `:help stapl`.


# vim-stapl
Some vimscript for stapl

# Installing
```
cd ~/.vim/plugin
git clone git@gitlab.com:parasol-lab/vim-stapl.git
```

Or if using vim-plug or a similar package manager:
```
Plug 'git@gitlab.com:parasol-lab/vim-stapl'
```

I've sometimes noticed the `:StaplCheckFormatting` command to be slow to respond, so adding the following post-update hook could be helpful:
```
Plug 'git@gitlab.com:parasol-lab/vim-stapl', { 'do': 'python -m compileall assets/' }
```

# Configuring
I'll get around to writing more about this here and in the help, but for now, I'll recommend you set these options in your .vimrc
```
let g:stapl_set_opts = 1
let g:stapl_try_fix_format = 1
let g:stapl_ctags_program = 'ctags --extra+=q'
let g:stapl_use_file_templates = 1
```
The second line will make `:StaplCheckFormatting` automatically try and fix minor issues *iff* you have
[Patch 7.4.566](https://raw.githubusercontent.com/vim/vim/master/runtime/doc/version8.txt). Note that
if you have that patch, you might also have `:cfdo` which you can use to do something like
`:cfdo ClangFormat` to fix more substantial issues.
The last line will cause new files to be initialized based on the template files in the `assets/` folder.
To use your own custom templates, you can do the following

```
let g:stapl_file_templates = {
  \   "cc": {
  \     "template": "~/my_template.cc",
  \     "post_template": "startinsert",
  \   },
  \   "gmake": {
        "template": ""
  \   },
  \ }
```

As this shows, you can also add a `"post_template"` ex command to run after everything else, in this case
starting you off in insert mode. Note that `""` effectively disables templates (including the execution
of `"post_template"`).

By default, with `g:stapl_use_file_templates == 1`, it only triggers for .cc, .hpp, and GNUmakefile. To
use this for other files, set it to 2 and add the appropriate entry for your file extensions.

# Suggestion
If you aren't using git (in which case, you should use [vim-fugitive](https://github.com/tpope/vim-fugitive) or similar) and aren't using a VC plugin, consider using [vim-smallsvn](https://github.tamu.edu/mynetidfornow/vim-smallsvn) for SVN with STAPL. All it has is support for svn blame and a few things for looking at svn diff output. Blame in particular turns out to be surprisingly useful in STAPL.
