#include <Aria/Aria.h>

#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    struct TestCase
    {
        const char *name;
        const char *category;
    };

    const TestCase kHardwarePrograms[] = {
        {"absoluteHeadingActionTest", "test"},
        {"simpleConnect", "example"},
        {"simpleMotionCommands", "example"},
        {"teleopActionsExample", "example"},
    };

    struct Result
    {
        std::string name;
        std::string category;
        std::string path;
        int exitCode;
        bool launched;
        bool passed;
    };

    std::string executableDirectory()
    {
        char path[4096];
        const ssize_t size = readlink("/proc/self/exe", path, sizeof(path) - 1);
        if (size <= 0)
        {
            return ".";
        }

        path[size] = '\0';
        std::string fullPath(path);
        const std::string::size_type slash = fullPath.find_last_of('/');
        if (slash == std::string::npos)
        {
            return ".";
        }
        return fullPath.substr(0, slash);
    }

    std::string shellQuote(const std::string &text)
    {
        std::string quoted("'");
        for (std::string::const_iterator it = text.begin(); it != text.end(); ++it)
        {
            if (*it == '\'')
            {
                quoted += "'\\''";
            }
            else
            {
                quoted += *it;
            }
        }
        quoted += "'";
        return quoted;
    }

    Result runProgram(const std::string &directory, const TestCase &testCase, const std::vector<std::string> &extraArgs)
    {
        Result result;
        result.name = testCase.name;
        result.category = testCase.category;
        result.path = directory + "/" + testCase.name;
        result.exitCode = -1;
        result.launched = false;
        result.passed = false;

        if (access(result.path.c_str(), X_OK) != 0)
        {
            return result;
        }

        std::string command = shellQuote(result.path);
        for (std::vector<std::string>::const_iterator it = extraArgs.begin(); it != extraArgs.end(); ++it)
        {
            command += " ";
            command += shellQuote(*it);
        }

        const int status = std::system(command.c_str());
        result.launched = true;

        if (status == -1)
        {
            result.exitCode = errno;
            return result;
        }

        if (WIFEXITED(status))
        {
            result.exitCode = WEXITSTATUS(status);
            result.passed = (result.exitCode == 0);
            return result;
        }

        if (WIFSIGNALED(status))
        {
            result.exitCode = 128 + WTERMSIG(status);
        }
        else
        {
            result.exitCode = status;
        }
        return result;
    }

    void writeReport(const std::string &reportPath, const std::vector<Result> &results)
    {
        std::ofstream report(reportPath.c_str());
        report << "libaria hardware test runner\n";
        report << "===========================\n\n";

        int passed = 0;
        for (std::vector<Result>::const_iterator it = results.begin(); it != results.end(); ++it)
        {
            if (it->passed)
            {
                ++passed;
            }
            report << std::left << std::setw(8) << (it->passed ? "PASS" : "FAIL")
                   << "  " << std::setw(8) << it->category
                   << "  " << it->name
                   << "  exit=" << it->exitCode
                   << "  path=" << it->path << "\n";
        }

        report << "\nSummary: " << passed << "/" << results.size() << " passed\n";
    }

    void printUsage(const char *programName)
    {
        std::cout << "Usage: " << programName << " [--report path] [-- <common robot args...>]\n"
                  << "Example:\n"
                  << "  " << programName << " --report hardware.txt -- -robotPort /dev/ttyUSB0\n"
                  << "  " << programName << " -- --remoteHost 192.168.0.10\n";
    }
}

int main(int argc, char **argv)
{
    std::string reportPath = "libaria_hardware_test_report.txt";
    std::vector<std::string> extraArgs;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--help" || arg == "-h")
        {
            printUsage(argv[0]);
            return 0;
        }
        if (arg == "--report" && i + 1 < argc)
        {
            reportPath = argv[++i];
            continue;
        }
        if (arg == "--")
        {
            for (++i; i < argc; ++i)
            {
                extraArgs.push_back(argv[i]);
            }
            break;
        }
    }

    const std::string directory = executableDirectory();
    std::vector<Result> results;

    std::cout << "Running hardware libaria tests from " << directory << "\n";
    if (!extraArgs.empty())
    {
        std::cout << "Common program arguments:";
        for (std::vector<std::string>::const_iterator it = extraArgs.begin(); it != extraArgs.end(); ++it)
        {
            std::cout << " " << *it;
        }
        std::cout << "\n";
    }

    for (size_t i = 0; i < sizeof(kHardwarePrograms) / sizeof(kHardwarePrograms[0]); ++i)
    {
        const Result result = runProgram(directory, kHardwarePrograms[i], extraArgs);
        results.push_back(result);

        std::cout << std::left << std::setw(8)
                  << (result.passed ? "PASS" : "FAIL")
                  << "  " << std::setw(8) << result.category
                  << "  " << result.name;
        if (!result.launched)
        {
            std::cout << " (not found or not executable)";
        }
        std::cout << " [exit=" << result.exitCode << "]\n";
    }

    writeReport(reportPath, results);
    std::cout << "Wrote report to " << reportPath << "\n";

    int failures = 0;
    for (std::vector<Result>::const_iterator it = results.begin(); it != results.end(); ++it)
    {
        if (!it->passed)
        {
            ++failures;
        }
    }

    std::cout << "Summary: " << (results.size() - failures) << "/" << results.size() << " passed\n";
    return failures == 0 ? 0 : 1;
}