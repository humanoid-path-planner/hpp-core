/**

\page hpp_core_porting_notes Porting notes

how to upgrade user's code when switching to a more recent version of hpp-core.

\section hpp_core_porting_3_1_to_3_2 Porting from version 3.1 to version 3.2

\subsection hpp_core_porting_3_1_to_3_2_report Modification in validation reports

Validation reports used to be passed by reference to validation methods. This
avoided unnecessary dynamic allocation since the user would create a report of
the right type and call the validation method. This strategy however is not
scalable to an extension of the hierarchy of validation methods and of
validation reports. For this reason the following modifications have been done.

The following methods related to configuration validation are deprecated

\li \code virtual bool ConfigValidation::validate (const Configuration_t& config, bool throwIfInValid) = 0 \endcode
\li \code virtual bool ConfigValidation::validate (const Configuration_t& config, ValidationReport& validationReport, bool throwIfInValid) = 0 \endcode

\li \code virtual bool ConfigValidations::validate (const Configuration_t& config, bool throwIfInValid) \endcode
\li \code virtual bool ConfigValidations::validate (const Configuration_t& config, ValidationReport& validationReport, bool throwIfInValid) \endcode

\li \code virtual bool CollisionValidation::validate (const Configuration_t& config, bool throwIfInValid) \endcode
\li \code virtual bool CollisionValidation::validate (const Configuration_t& config, ValidationReport& validationReport, bool throwIfInValid) \endcode

\li \code virtual bool JointBoundValidation::validate (const Configuration_t& config, bool throwIfInValid) \endcode
\li \code virtual bool JointBoundValidation::validate (const Configuration_t& config, ValidationReport& validationReport, bool throwIfInValid) \endcode

Methods taking a \c ValidationReportPtr_t& as second argument should be used instead.

The following methods related to path validation are deprecated:

\li \code virtual bool PathValidation::validate (const PathPtr_t& path, bool reverse, PathPtr_t& validPart) = 0 \endcode
\li \code virtual bool PathValidation::validate (const PathPtr_t& path, bool reverse, PathPtr_t& validPart, ValidationReport& report) = 0 \endcode

\li \code virtual bool DiscretizedCollisionChecking::validate (const PathPtr_t& path, bool reverse, PathPtr_t& validPart) \endcode
\li \code virtual bool DiscretizedCollisionChecking::validate (const PathPtr_t& path, bool reverse, PathPtr_t& validPart, ValidationReport& validationReport) \endcode

\li \code virtual bool continuousCollisionChecking::Dichotomy::validate (const PathPtr_t& path, bool reverse, PathPtr_t& validPart) \endcode
\li \code virtual bool continuousCollisionChecking::Dichotomy::validate (const PathPtr_t& path, bool reverse, PathPtr_t& validPart, ValidationReport& validationReport) \endcode

\li \code virtual bool continuousCollisionChecking::Progressive::validate (const PathPtr_t& path, bool reverse, PathPtr_t& validPart) \endcode
\li \code virtual bool continuousCollisionChecking::Progressive::validate (const PathPtr_t& path, bool reverse, PathPtr_t& validPart, ValidationReport& validationReport) \endcode

Methods taking a reference to a \c PathValidationReportPtr_t as fourth argument should be used instead.

 */
