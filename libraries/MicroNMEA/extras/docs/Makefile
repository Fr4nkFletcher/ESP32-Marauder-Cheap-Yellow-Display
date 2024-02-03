RMDIR = rm -rf --

default: docs

.PHONY: docs
docs:
	doxygen Doxyfile 
	sphinx-build -b html . _build/html

.PHONY: clean
clean:
	$(RMDIR) _build xml

