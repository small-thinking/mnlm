#!/bin/bash

# Function to check the status of the last command
check_status() {
    if [ $? -ne 0 ]; then
        echo "Error: $1"
        exit 1
    fi
}

# Setup bashrc
echo "Setting up bashrc..."
cp ./mnlm/resources/.bashrc .bashrc
cat ./.bashrc >> ~/.bashrc

# Install pathogen
pushd /home/small-thinking

echo "Installing pathogen..."
mkdir -p /home/small-thinking/.vim/
cp ./mnlm/resources/.vimrc .vimrc
mkdir -p .vim/autoload ./.vim/bundle && \
curl -LSso .vim/autoload/pathogen.vim https://tpo.pe/pathogen.vim
check_status "Failed to install pathogen."

# Install solarized theme
echo "Installing solarized theme..."
mkdir -p .vim/colors/ && mv ./mnlm/resources/solarized.vim .vim/colors/solarized.vim
# git clone git://github.com/altercation/vim-colors-solarized.git
# mv vim-colors-solarized .vim/bundle/
check_status "Failed to install solarized theme."

# Install vim plugins
echo "Installing vim plugins..."


# Cleanup
popd
echo "Finished setup."

