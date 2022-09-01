#!/bin/bash

# vision files need the "specialType" of "vision_config"
# robot config files need the "specialType" of "device_config"

# Search for source/header files, excluding those in the .git directory
files=$(find -name "*.cpp" |grep -v "git" |sed "s|./||")
files="${files} $(find -name "*.h" |grep -v "git" |sed "s|./||")"
files="${files} $(find -name "*.md" |grep -v "git" |sed "s|./||")"

# Search for directories under src, include or core
directories=$(find -type d -regex "./\(core\|src\|include\)/.*" |sed "s|./||")

output=$(mktemp)

# generate the "File" type of json
for i in ${files}; do
	special=""
	if [[ -n "$(echo "$i" |grep "robot-config")" ]]; then
		special="device_config"
	elif [[ -n "$(egrep "vex::vision [^=]*=" $i)" ]]; then
		special="vision_config"
	fi

	builder="{\"name\": \"$i\", \"type\": \"File\", \"specialType\":\"$special\"},"
	echo "${builder}" >> $output
done

# generate the "Folder" type of json
for i in ${directories}; do
	builder="{\"name\": \"$i\", \"type\": \"Directory\"},"
	echo "${builder}" >> $output
done

sed -i "$ s/,$//" $output

output_var="$(cat $output |tr -d "[:space:]")"
sed -i "s|\"files\":\[[^]]*|\"files\":[$output_var|" *.v5code 

#cat $output
