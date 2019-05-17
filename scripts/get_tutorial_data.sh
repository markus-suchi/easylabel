#!/bin/bash

echo "Downloading TUW model database..."
output=$(wget -c -O easylabel_tutorial_data.tar.gz https://data.acin.tuwien.ac.at/index.php/s/5PdlJAftADmXMsq/download)
if [ $? -ne 0 ]; then
    echo "Error downloading file"
else
    echo "File has been downloaded"
    echo "Inflating file..."

    if ! tar -zxvf ./easylabel_tutorial_data.tar.gz &> /dev/null; then
        echo "Failure during inflating.."
    else
        echo "Successfully inflated file! Deleting tar file..."
        rm easylabel_tutorial_data.tar.gz

        mkdir data_tutorial/accu        
        mkdir data_tutorial/label          
        echo "Done!"
    fi
fi

