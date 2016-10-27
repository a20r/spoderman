AM_CPPFLAGS = $(CXX11_FLAGS) -I$(srcdir)/../src
AM_CXXFLAGS = $(PICKY_CXXFLAGS)
LDADD = ../src/libsourdough.a -lpthread -lgsl -lgslcblas -lm
PICKY_CXXFLAGS = -g

common_source = contest_message.hh contest_message.cc \
	controller.hh controller.cc rate_filter.cpp rate_filter.hpp

bin_PROGRAMS = sender receiver


sender_SOURCES = $(common_source) sender.cc

receiver_SOURCES = $(common_source) receiver.cc
