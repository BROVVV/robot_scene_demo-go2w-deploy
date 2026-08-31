#! /bin/bash

grep  \begin{align} *tex > /tmp/log.log
cat -n /tmp/log.log
rm /tmp/log.log
