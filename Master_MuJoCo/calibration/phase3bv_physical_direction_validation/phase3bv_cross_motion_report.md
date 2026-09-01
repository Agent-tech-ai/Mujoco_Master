# Phase 3B-V cross-motion report

## Per-motion aggregate

| motion | channels | baseline_mean_abs_error_rad | mass_mean_abs_error_rad | mean_abs_error_improvement_percent |
|---|---|---|---|---|
| clap | 12 | 0.0182293 | 0.0165219 | 9.36628 |
| heart | 12 | 0.0113304 | 0.0112881 | 0.372803 |
| wave | 12 | 0.0230266 | 0.0209107 | 9.18892 |

Heart/Wave rows are read from the immutable Phase 3B-S comparison artifact; no prior replay was altered or rerun.

## Per-channel matrix

| motion | channel | real_excursion_rad | baseline_sim_excursion_rad | mass_sim_excursion_rad | baseline_abs_error_rad | mass_abs_error_rad | percent_error_improvement | rmse_change_rad | velocity_rmse_change_rad_s | xcorr_lag_change_s | safety |
|---|---|---|---|---|---|---|---|---|---|---|---|
| heart | left_ankle_pitch_joint | 0.0461992 | 0.03494 | 0.0353732 | 0.0112592 | 0.0108261 | 3.84713 | -0.00034831 | -0.00137908 | 0 | PRESERVED_PHASE3BS |
| heart | left_ankle_roll_joint | 0.00402641 | 0.00402379 | 0.00392675 | 2.6188e-06 | 9.96643e-05 | -3705.72 | 5.09961e-05 | 3.95662e-05 | 0 | PRESERVED_PHASE3BS |
| heart | left_hip_pitch_joint | 0.0356567 | 0.029902 | 0.02729 | 0.00575471 | 0.00836668 | -45.3884 | -0.00132517 | -0.00261737 | 0 | PRESERVED_PHASE3BS |
| heart | left_hip_roll_joint | 0.00342557 | 0.00225059 | 0.00231847 | 0.00117498 | 0.0011071 | 5.77693 | -0.000175504 | 2.04821e-05 | 0 | PRESERVED_PHASE3BS |
| heart | left_knee_joint | 0.00747776 | 0.0560195 | 0.0520083 | 0.0485417 | 0.0445305 | 8.26345 | -0.00194712 | -0.00340068 | -0.02 | PRESERVED_PHASE3BS |
| heart | right_ankle_pitch_joint | 0.0320215 | 0.0392841 | 0.0394426 | 0.00726253 | 0.00742104 | -2.18255 | -8.85553e-05 | -0.00194297 | 0 | PRESERVED_PHASE3BS |
| heart | right_ankle_roll_joint | 0.0024929 | 0.00795096 | 0.00773934 | 0.00545806 | 0.00524644 | 3.87718 | 8.76133e-05 | 4.16198e-05 | 0 | PRESERVED_PHASE3BS |
| heart | right_hip_pitch_joint | 0.0410337 | 0.0314691 | 0.0291656 | 0.00956461 | 0.0118682 | -24.0841 | -0.00124741 | -0.00233644 | 0 | PRESERVED_PHASE3BS |
| heart | right_hip_roll_joint | 0.00265698 | 0.00527973 | 0.00517572 | 0.00262275 | 0.00251874 | 3.96588 | 0.000148332 | 3.14541e-05 | 0 | PRESERVED_PHASE3BS |
| heart | right_knee_joint | 0.0237765 | 0.054343 | 0.0498275 | 0.0305664 | 0.026051 | 14.7725 | -0.000941725 | -0.00245439 | -0.04 | PRESERVED_PHASE3BS |
| heart | waist_pitch_joint | 0.0323393 | 0.0206345 | 0.016985 | 0.0117049 | 0.0153543 | -31.1786 | -0.000393203 | -0.00159048 | 0 | PRESERVED_PHASE3BS |
| heart | waist_roll_joint | 0.01018 | 0.00812792 | 0.00811199 | 0.00205203 | 0.00206797 | -0.77661 | 2.36742e-05 | 1.18617e-05 | -0.02 | PRESERVED_PHASE3BS |
| wave | left_ankle_pitch_joint | 0.0301042 | 0.0241447 | 0.0229228 | 0.00595949 | 0.00718133 | -20.5024 | -0.000647925 | -0.00287721 | -0.12 | PRESERVED_PHASE3BS |
| wave | left_ankle_roll_joint | 0.00882101 | 0.023587 | 0.0198804 | 0.014766 | 0.0110594 | 25.1021 | -0.00312213 | -0.00284978 | -0.02 | PRESERVED_PHASE3BS |
| wave | left_hip_pitch_joint | 0.0139976 | 0.0489201 | 0.0464232 | 0.0349226 | 0.0324256 | 7.14992 | -0.00211057 | -0.00104767 | -0.02 | PRESERVED_PHASE3BS |
| wave | left_hip_roll_joint | 0.00997114 | 0.0145798 | 0.0124015 | 0.00460869 | 0.00243034 | 47.2663 | -0.00188082 | -0.00131726 | 0 | PRESERVED_PHASE3BS |
| wave | left_knee_joint | 0.00728607 | 0.0723737 | 0.0705045 | 0.0650876 | 0.0632184 | 2.87191 | 0.00145496 | 0.000178896 | -0.06 | PRESERVED_PHASE3BS |
| wave | right_ankle_pitch_joint | 0.0279946 | 0.0157867 | 0.0158186 | 0.0122079 | 0.012176 | 0.261396 | 0.00117832 | 0.00161097 | -0.08 | PRESERVED_PHASE3BS |
| wave | right_ankle_roll_joint | 0.0093956 | 0.0245507 | 0.0210421 | 0.0151551 | 0.0116465 | 23.1509 | -0.00225142 | -0.0032916 | -0.14 | PRESERVED_PHASE3BS |
| wave | right_hip_pitch_joint | 0.0170655 | 0.0431003 | 0.0412701 | 0.0260348 | 0.0242046 | 7.02993 | -0.000893993 | -0.000339155 | -0.02 | PRESERVED_PHASE3BS |
| wave | right_hip_roll_joint | 0.00958776 | 0.0166444 | 0.0123558 | 0.00705662 | 0.00276802 | 60.7741 | 0.000809559 | -0.00141707 | 1.5 | PRESERVED_PHASE3BS |
| wave | right_knee_joint | 0.0101536 | 0.082092 | 0.0761836 | 0.0719384 | 0.0660299 | 8.21322 | -0.00170447 | -0.00253854 | 0 | PRESERVED_PHASE3BS |
| wave | waist_pitch_joint | 0.0280406 | 0.0298661 | 0.0277178 | 0.0018255 | 0.000322743 | 82.3202 | -0.000174433 | -0.000288678 | -1.1 | PRESERVED_PHASE3BS |
| wave | waist_roll_joint | 0.0386269 | 0.0218709 | 0.0211618 | 0.016756 | 0.0174651 | -4.23173 | 2.58591e-05 | 0.00163662 | 0 | PRESERVED_PHASE3BS |
| clap | left_ankle_pitch_joint | 0.0358562 | 0.0684663 | 0.0655796 | 0.03261 | 0.0297234 | 8.85211 | -0.00115699 | -0.00120959 | 0.02 | COMPARATIVE_PRESERVED_ABSOLUTE_FAIL |
| clap | left_ankle_roll_joint | 0.00210905 | 0.00396424 | 0.0052486 | 0.00185519 | 0.00313955 | -69.2307 | 0.000903753 | -4.05172e-05 | -0.62 | COMPARATIVE_PRESERVED_ABSOLUTE_FAIL |
| clap | left_hip_pitch_joint | 0.0226259 | 0.0498687 | 0.0464227 | 0.0272428 | 0.0237967 | 12.6493 | -0.00237755 | -0.00133699 | -0.02 | COMPARATIVE_PRESERVED_ABSOLUTE_FAIL |
| clap | left_hip_roll_joint | 0.00245947 | 0.00325044 | 0.00371507 | 0.000790974 | 0.0012556 | -58.7407 | 0.000619867 | -4.29617e-05 | 0 | COMPARATIVE_PRESERVED_ABSOLUTE_FAIL |
| clap | left_knee_joint | 0.00651932 | 0.0460205 | 0.0407204 | 0.0395012 | 0.0342011 | 13.4177 | -0.00176754 | -0.00248582 | 0 | COMPARATIVE_PRESERVED_ABSOLUTE_FAIL |
| clap | right_ankle_pitch_joint | 0.0333638 | 0.0716727 | 0.069386 | 0.0383089 | 0.0360222 | 5.96912 | -0.00118546 | -0.000421599 | 0.04 | COMPARATIVE_PRESERVED_ABSOLUTE_FAIL |
| clap | right_ankle_roll_joint | 0.00210905 | 0.00354844 | 0.00301277 | 0.00143939 | 0.000903717 | 37.2151 | -0.000768967 | -3.50311e-05 | -1.74 | COMPARATIVE_PRESERVED_ABSOLUTE_FAIL |
| clap | right_hip_pitch_joint | 0.021667 | 0.0530528 | 0.0515862 | 0.0313858 | 0.0299192 | 4.67257 | -0.00181424 | -0.00109652 | 0.02 | COMPARATIVE_PRESERVED_ABSOLUTE_FAIL |
| clap | right_hip_roll_joint | 0.00210953 | 0.00484989 | 0.0036209 | 0.00274036 | 0.00151137 | 44.8476 | -0.000642504 | 3.80554e-05 | -0.02 | COMPARATIVE_PRESERVED_ABSOLUTE_FAIL |
| clap | right_knee_joint | 0.00805378 | 0.047824 | 0.0427505 | 0.0397702 | 0.0346967 | 12.7569 | -0.0024344 | -0.0033115 | -0.02 | COMPARATIVE_PRESERVED_ABSOLUTE_FAIL |
| clap | waist_pitch_joint | 0.00736861 | 0.00469274 | 0.00451862 | 0.00267587 | 0.00285 | -6.50721 | -4.13262e-06 | 1.91672e-05 | 0 | COMPARATIVE_PRESERVED_ABSOLUTE_FAIL |
| clap | waist_roll_joint | 0.00701887 | 0.00744944 | 0.00726166 | 0.00043057 | 0.000242784 | 43.6135 | -0.000155573 | -9.68657e-05 | -0.02 | COMPARATIVE_PRESERVED_ABSOLUTE_FAIL |

Wave right-knee remains: real `0.0101536374 rad`, baseline simulation `0.0820920169 rad`,
ratio `8.084986×`, absolute error `0.0719383795 rad`. Absolute error is the decision metric.
