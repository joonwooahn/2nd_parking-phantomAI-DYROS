#pragma once

class Filter
{
public:
	Filter ();
	~Filter ();

	void HDR (double v[3], double threshold, double ic);
	void HSR (double v[3], double threshold, double ic);
	void LowpPassFilter (double v[3]);
	void Delagging (double v[3]);

private:
	double _T;
	double _tau;

	double _HDR_I[3];
	double _HSR_I;

	double _w_lp[3];
	double _w_dl[3];

	bool _w_lp_init;
	bool _w_dl_init;
};