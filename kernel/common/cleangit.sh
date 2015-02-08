for I in `git-wkm.sh`
do
	echo $I
	scripts/Lindent $I
	perl -pi -e 's/  *$/* g;s/\t*$//g;s/\/\/ (.*)/\/\* \1 \*\//g;s/\/\/\n//g;s/\/\/([^w][^w].*)/\/\* \1 \*\//g' $I */
done
