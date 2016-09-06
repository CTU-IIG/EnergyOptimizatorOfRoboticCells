/*
	This file is part of the EnergyOptimizatorOfRoboticCells program.

	EnergyOptimizatorOfRoboticCells is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	EnergyOptimizatorOfRoboticCells is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with EnergyOptimizatorOfRoboticCells. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Settings.h"
#include "HeuristicSolver/KnowledgeBase.h"

using namespace std;

KnowledgeBase::KnowledgeBase() : mAddedTuples(0u), mProcessedTuples(0u), mOptimizationPhaseCounter(0u), mInfDueToLP(0u),
	mInfDueToLocHeur(0u), mInfDueToPwrmHeur(0u), mInfDueToChangedPath(0u), mInfeasibleCounter(0u), mLPCalls(0u), mLPFixCalls(0u),
	mPartProbCalls(0u), mLocHeurCalls(0u), mPwrmHeurCalls(0u), mPathChangeCalls(0u), mSumOfIters(0u), mLPTime(0.0), mLocHeurTime(0.0),
	mPwrmHeurTime(0.0), mTupleGenTime(0.0), mSumOfDeteriorations(0u), mSumOfLocHeurErrs(0.0), mSumOfPwrmHeurErrs(0.0) { }

list<pair<Solution, CircuitTuple>> KnowledgeBase::eliteSolutions() {
	mEliteMtx.lock();
	list<pair<Solution, CircuitTuple>> sols = mEliteSolutions;
	mEliteMtx.unlock();
	return sols;
}

Solution KnowledgeBase::bestSolution()	{
	Solution best;
	mEliteMtx.lock();
	if (!mEliteSolutions.empty())	{
		best = mEliteSolutions.front().first;
		mEliteMtx.unlock();
	} else {
		mEliteMtx.unlock();
		throw EmptySolutionPool(caller(), "No feasible solution found!");
	}

	return best;
}

void KnowledgeBase::candidate(const Solution& s, const CircuitTuple& t)	{

	if (s.status == INFEASIBLE || s.status == UNKNOWN)
		return;

	mEliteMtx.lock();

	if (!mEliteSolutions.empty())	{
		if (s.totalEnergy >= mEliteSolutions.back().first.totalEnergy) {
			if (mEliteSolutions.size() < Settings::MAX_ELITE_SOLUTIONS)
				mEliteSolutions.push_back({s, t});
		} else {
			list<pair<Solution, CircuitTuple>>::iterator it = mEliteSolutions.begin(), eit = mEliteSolutions.end();
			while (it != eit && it->first.totalEnergy <= s.totalEnergy) {
				++it;
			}
			mEliteSolutions.insert(it, {s, t});
			if (mEliteSolutions.size() > Settings::MAX_ELITE_SOLUTIONS)
				mEliteSolutions.pop_back();
		}
	} else {
		mEliteSolutions.push_back({s, t});
	}

	mEliteMtx.unlock();
}

void KnowledgeBase::addTuple(CircuitTuple&& t)	{
	mTuplesMtx.lock();
	mTuples.push(move(t));
	mTuplesMtx.unlock();
	++mAddedTuples;
}

CircuitTuple KnowledgeBase::getTuple()	{
	CircuitTuple tuple;
	mTuplesMtx.lock();
	if (!mTuples.empty())	{
		tuple = mTuples.front(); mTuples.pop();
		mTuplesMtx.unlock();
	} else {
		mTuplesMtx.unlock();
		throw EmptySolutionPool(caller(), "No tuples are ready to be processed!");
	}

	++mProcessedTuples;

	return tuple;
}

void KnowledgeBase::addErrorMessage(const string& msg)	{
	mErrorsMtx.lock();
	mErrorMessages.push_back(msg);
	mErrorsMtx.unlock();
}

void KnowledgeBase::recordInfeasibleLP() {
	++mInfeasibleCounter;
}

double KnowledgeBase::infeasibilityRate()	{
	uint64_t calls = mLPCalls, infResults = mInfeasibleCounter;
	if (calls > 0)
		return (((double) infResults)/((double) calls));
	else
		return 0.0;
}

void KnowledgeBase::recordLPCall(double runtime)	{
	record(runtime, mLPTime, mLPCalls);
}

pair<uint64_t, double> KnowledgeBase::infoLP() {
	return getInfo(mLPTime, mLPCalls);
}

void KnowledgeBase::recordLPFixDeterioration(double deterioration)	{
	record(deterioration, mSumOfDeteriorations, mLPFixCalls);
}

double KnowledgeBase::averageLPFixDeterioration()	{
	return getInfo(mSumOfDeteriorations, mLPFixCalls).second;
}

double KnowledgeBase::averageNumberOfLPCallsForLPFix()	{
	uint64_t numOfLPCalls = mLPCalls, numOfLPFixCalls = mLPFixCalls;
	if (mLPFixCalls > 0)
		return (((double) numOfLPCalls)/((double) numOfLPFixCalls));
	else
		return 1.0;
}

void KnowledgeBase::recordPartialProblemSolveCall()	{
	++mPartProbCalls;
}

void KnowledgeBase::recordLocHeurCall(double runtime)	{
	record(runtime, mLocHeurTime, mLocHeurCalls);
}

pair<uint64_t, double> KnowledgeBase::infoLocHeur()	{
	return getInfo(mLocHeurTime, mLocHeurCalls);
}

void KnowledgeBase::recordLocHeurRelErr(double relativeEstError)	{
	record(relativeEstError, mSumOfLocHeurErrs);
}

double KnowledgeBase::averageErrOfLocHeur()	{
	return getInfo(mSumOfLocHeurErrs, mLocHeurCalls).second;
}

void KnowledgeBase::recordPwrmHeurCall(double runtime)	{
	record(runtime, mPwrmHeurTime, mPwrmHeurCalls);
}

pair<uint64_t, double> KnowledgeBase::infoPwrmHeur()	{
	return getInfo(mPwrmHeurTime, mPwrmHeurCalls);
}

void KnowledgeBase::recordPwrmHeurRelErr(double relativeEstError)	{
	record(relativeEstError, mSumOfPwrmHeurErrs);
}

double KnowledgeBase::averageErrOfPwrmHeur()	{
	return getInfo(mSumOfPwrmHeurErrs, mPwrmHeurCalls).second;
}

void KnowledgeBase::recordAddTuplesCall(double runtime)	{
	record(runtime, mTupleGenTime);
}

pair<uint64_t, double> KnowledgeBase::infoTuplesGeneration()	{
	return getInfo(mTupleGenTime, mAddedTuples);
}

void KnowledgeBase::recordNumberOfItersPerTuple(uint64_t iters)	{
	mSumOfIters += iters;
	++mOptimizationPhaseCounter;
}

pair<uint64_t, double> KnowledgeBase::infoItersPerTuple()	{
	uint64_t consideredTuples = mOptimizationPhaseCounter, sumOfIters = mSumOfIters;
	if (consideredTuples > 0)
		return {consideredTuples, ((double) sumOfIters)/((double) consideredTuples)};
	else
		return {0u, 0.0};
}

double KnowledgeBase::percentageOfProcessed()	{
	uint64_t processed = mProcessedTuples, added = mAddedTuples;
	if (added > 0)
		return 100.0*(((double) processed)/((double) added));
	else
		return 100.0;
}

template <class T>
extern void KnowledgeBase::record(T value, T& addTo)	{
	mStatMtx.lock();
	addTo += value;
	mStatMtx.unlock();
}

template <class T>
extern void KnowledgeBase::record(T value, T& addTo, std::atomic<uint64_t>& counter)	{
	record(value, addTo);
	++counter;
}

pair<uint64_t, double> KnowledgeBase::getInfo(double& aggregatedValue, std::atomic<uint64_t>& counter)	{
	mStatMtx.lock();
	double totalValue = aggregatedValue;
	uint64_t numCalls = counter;
	mStatMtx.unlock();
	if (numCalls > 0)
		return {numCalls, totalValue/numCalls};
	else
		return {numCalls, 0.0};
}

