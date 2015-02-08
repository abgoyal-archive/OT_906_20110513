git status | egrep -i "modified" | awk '{printf "%s\n", $3}'
git status | egrep -i "new\ file" | awk '{printf "%s\n", $4}'
