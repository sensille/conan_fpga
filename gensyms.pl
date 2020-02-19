#!/usr/bin/perl -w

use strict;

if (@ARGV != 3) {
	die "usage: gensyms.pl <modelname> <src-h> <dst-h>\n";
}
my ($model, $srcname, $dstname) = @ARGV;

open (my $src, '<', $srcname)
	or die "failed to open $srcname";
open (my $dst, '>', $dstname)
	or die "failed to open $dstname";

# QData/*34:0*/ conan__DOT__u_command__DOT__rcv_param;
# QData/*63:0*/ conan__DOT__u_led7219__DOT__dout;
# CData/*7:0*/ conan__DOT__u_framing__DOT__recv_ring[256];

while (<$src>) {
	last if (/^\s+\/\/ LOCAL SIGNALS$/);
}
my @signals;
while (<$src>) {
	last if (/^\s+\/\/ LOCAL VARIABLES$/);
	next if (/^\s*$/);
	next if (/^\s*\/\//);
	next if (/^\s*};/);
	next if (/^\s*struct \{/);
	if (!/^\s+([CIQSW])Data\/\*(\d+):(\d+)\*\/ (\w+)(\[\d+\])?;$/) {
		die "failed to parse $_";
	}
	my $type = $1;
	my $range_start = $2;
	my $range_end = $3;
	my $name_raw = $4;
	my $num = $5 // 1;
	$num =~ s/[\[\]]//g;
	my $name_cooked = $name_raw;
	$name_cooked =~ s/__DOT__/./g;
	# first component is always the same, strip for convenience
	$name_cooked =~ s/^$model\.//;
	my $type = "sig" . $type;

	push @signals, {
		type => $type,
		range_start => $range_start,
		range_end => $range_end,
		name_raw => $name_raw,
		name_cooked => $name_cooked,
		num => $num,
	};
}
print $dst <<"HERE";
#ifndef __VSYMS__H__
#define __VSYMS__H__

#include <verilated.h>

typedef enum _sigtype {
	sigC,
	sigI,
	sigQ,
	sigS,
	sigW
} sigtype_t;

typedef struct _signal {
	const char	*name;		/* cooked name */
	sigtype_t	type;
	int		range_start;	/* bitfield range */
	int		range_end;
	int		num;		/* number of entries in array */
	int		offset;		/* offset of signal in structure */
} signal_t;

signal_t vsigs[] = {
HERE

for (@signals) {
	print $dst "\t{ \"$_->{name_cooked}\", $_->{type}, $_->{range_start}, ";
	print $dst "$_->{range_end}, $_->{num}, offsetof(Vconan, $_->{name_raw}) },\n";
}

print $dst <<"HERE";
};
#define NSIGS (sizeof(vsigs) / sizeof(signal_t))

#endif
HERE
